#!/usr/bin/env python

'''
Runner Node
Author: Curt Henrichs
Date: 5-28-19

ITER's main application node.

Runner node provides services to run a fully defined job as a JSON string and
interacts with environment, timing, and respectiv primitive nodes to coordinate
the job.

Services provided:
    - /runner/task_input
    - /runner/get_mode
    - /runner/set_mode

Services requested:
    - /environment/set_vision_params
    - /environment/generate_task_objects
    - /environment/clear_task_objects
    - /environment/connect_task_object
    - /environment/release_task_object
    - /environment/get_vision_object
    - /environment/calibrate_robot_to_camera
    - /environment/get_state

Topics published:
    - /time_node/start
    - /time_node/stop
    - /time_node/sync

Topics subscribed:
    - /button/pressed
rrikik
Parameters required:
    - use_rik
    - use_rik_static
    - ~start_delay_time

'''

import sys
import time
import json
import rospy
import traceback

from iter_app_tools.pose_conversion import *
from geometry_msgs.msg import Vector3
from iter_app.msg import EnvironmentObject
from std_msgs.msg import String, Int32, Bool
from iter_app_tools.time_mode_enum import TimeModeEnum
from iter_app_tools.environment_client import EnvironmentClient
from iter_app.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse
from iter_app_tools.primitives.abstract import Primitive, ReturnablePrimitive

rospy.init_node('runner')
envClient = EnvironmentClient()

from iter_app_tools.primitives.default import DefaultBehaviorPrimitives

planner = rospy.get_param('planner','moveit')
use_static = rospy.get_param('use_static',False)
if planner == 'rik_python' or planner == 'rik_julia':
    if use_static:
        from iter_app_tools.primitives.relaxedik_static import RelaxedIKStaticBehaviorPrimitives as PhysicalBehaviorPrimitives
        from iter_app_tools.primitives.relaxedik_static import initialize_robot
    else:
        from iter_app_tools.primitives.relaxedik_realtime import RelaxedIKRealTimeBehaviorPrimitives as PhysicalBehaviorPrimitives
        from iter_app_tools.primitives.relaxedik_realtime import initialize_robot
elif planner == 'ur':
    from iter_app_tools.primitives.ur_direct import URDirectBehaviorPrimitives as PhysicalBehaviorPrimitives
    from iter_app_tools.primitives.ur_direct import initialize_robot
elif planner == 'moveit':
    from iter_app_tools.primitives.moveit import MoveItBehaviorPrimitives as PhysicalBehaviorPrimitives
    from iter_app_tools.primitives.moveit import initialize_robot
else:
    raise Exception('Invalid planner selected')

from iter_app_tools.primitives.general_movement import GeneralMovementBehaviorPrimitives
from iter_app_tools.primitives.environment_aware import EnvironmentAwareBehaviorPrimitives


bp = EnvironmentAwareBehaviorPrimitives(envClient=envClient,
     parent=GeneralMovementBehaviorPrimitives(
     parent=PhysicalBehaviorPrimitives(
     parent=DefaultBehaviorPrimitives())))


DEFAULT_NODE_START_DELAY_TIME = 5
DEFAULT_NODE_INITIALIZE_DELAY_TIME = 2

class Runner:

    def __init__(self):
        self._button_state = False
        self.time_mode = TimeModeEnum.REPLAY

        self.task_input_srv = rospy.Service('/runner/task_input', Task, self.run_task)
        self.mode_set_srv = rospy.Service('/runner/set/mode', ModeSet, self.mode_update)
        self.mode_get_srv = rospy.Service('/runner/get/mode', ModeGet, self.mode_get)

        self.time_start_topic = rospy.Publisher('/time_node/start', String, queue_size=10)
        self.time_stop_topic = rospy.Publisher('/time_node/stop', Bool, queue_size=10)
        self.time_sync_topic = rospy.Publisher('/time_node/sync', Int32, queue_size=10)

        self.btn_topic = rospy.Subscriber('/button/pressed',Bool,self._btn_topic_cb)

    def run_task(self, json_string):
        global envClient, bp
        self._button_state = False

        # data retrieved from interface
        try:
            data = json.loads(json_string.task_json)
        except:
            traceback.print_exc()
            return TaskResponse(False,'{}','Error parsing json stringified task')

        # if environment requested, load it
        if 'environment' in data.keys():
            objects = [self._env_obj_pkg(dct) for dct in data['environment']]
            resp = envClient.generate_task_objects(objects)
            if not resp.status:
                return TaskResponse(False,'{}','Failed to load environment')

        print 'Instantiating'
        primitives = []
        for obj in data['task']:
            primitives.append({
                'action': bp.instantiate_from_dict(obj,button_callback=self._button_callback),
                'is_interaction': 'rad' in obj and obj['rad']['is_interaction']})
        neglect_time_list = self._generate_neglect_time_list(data)

        rospy.sleep(1)

        # provide timing information to timing node
        if self.time_mode == TimeModeEnum.REPLAY:
            timeline = json.dumps(neglect_time_list)
            self.time_start_topic.publish(String(timeline))

        # iterate over all primitives
        print 'Running'
        operate_status = True
        was_interaction = None
        interactionCount = 0
        for index in range(0,len(primitives)):
            print type(primitives[index]['action']).__name__

            self._button_state = False # ignore any button press that happened in the interim

            if self.time_mode == TimeModeEnum.CAPTURE:

                start = time.time()

                if isinstance(primitives[index]['action'],ReturnablePrimitive):
                    operate_status, result = primitives[index]['action'].operate()
                else:
                    operate_status = primitives[index]['action'].operate()

                stop = time.time()

                if type(primitives[index]['action']) is bp.lookup('wait') and primitives[index]['action'].condition == 'button':
                    data['task'][index]['rad'] = {
                        'is_interaction': True,
                        'expected_interaction_time': stop - start
                    }
                else:
                    data['task'][index]['rad'] = {
                        'neglect_time': stop - start,
                        'is_interaction': False
                    }
            else:

                if was_interaction != primitives[index]['is_interaction']:
                    self.time_sync_topic.publish(Int32(interactionCount))
                    interactionCount += 1
                    was_interaction = primitives[index]['is_interaction']

                if isinstance(primitives[index]['action'],ReturnablePrimitive):
                    operate_status, result = primitives[index]['action'].operate()
                else:
                    operate_status = primitives[index]['action'].operate()

            if not operate_status:
                break

        # stop timing
        if self.time_mode == TimeModeEnum.REPLAY:
            rospy.sleep(0.25)
            self.time_stop_topic.publish(Bool(True))

        # clear environment resources
        responseMsg = ''
        if 'environment' in data.keys():
            clear_status = envClient.clear_task_objects([],True).status

            if not operate_status:
                responseMsg = 'failed during task execution'
            elif not clear_status:
                responseMsg = 'failed during environment teardown'

            operate_status = operate_status and clear_status

        # send results back to interface
        if self.time_mode == TimeModeEnum.CAPTURE:
            return TaskResponse(operate_status,json.dumps(data),responseMsg)
        else:
            return TaskResponse(operate_status,'{}',responseMsg)

    def mode_update(self, data):
        mode = TimeModeEnum.from_str(data.mode)
        if mode == None:
            raise Exception('Invalid mode supplied')

        self.time_mode = mode
        return ModeSetResponse()

    def mode_get(self, data):
        return ModeGetResponse(self.time_mode.value)

    def _generate_neglect_time_list(self,task_dict):
        time_list = []
        temp_neglect_time = 0

        for obj in task_dict['task']:
            if 'rad' in obj.keys():
                if 'is_interaction' in obj['rad'] and obj['rad']['is_interaction']: # push time to list
                    time_list.append({"time": temp_neglect_time, 'interaction': False})
                    temp_neglect_time = 0
                    time_list.append({"interaction": True, "time": obj['rad']['expected_interaction_time']})
                elif 'neglect_time' in obj['rad']: # increment by amount
                    temp_neglect_time += obj['rad']['neglect_time']

        time_list.append({"time": temp_neglect_time, 'interaction': False})

        return time_list

    def _env_obj_pkg(self,obj_dct):
        return EnvironmentObject(representation=EnvironmentObject.REPRESENTATION_BOX,
                                 id=obj_dct['name'],
                                 size=Vector3(x=obj_dct['size']['x'],
                                              y=obj_dct['size']['y'],
                                              z=obj_dct['size']['z']),
                                 pose=pose_dct_to_msg(obj_dct))

    def _button_callback(self):
        state = False
        if self._button_state:
            state = True
            self._button_state = False
        return state

    def _btn_topic_cb(self, message):
        if message.data:
            self._button_state = True


if __name__ == '__main__':

    start_delay_time = rospy.get_param('~start_delay_time',DEFAULT_NODE_START_DELAY_TIME)
    initialize_delay_time = rospy.get_param('~initialize_delay_time',DEFAULT_NODE_INITIALIZE_DELAY_TIME)

    rospy.sleep(start_delay_time)
    print "\n\nRunner is Starting\n\n"

    runner = Runner()

    rospy.sleep(initialize_delay_time)
    print '\n\nInitializing Robot\n\n'
    initialize_robot()

    print "\n\nRunner is Ready\n\n"
    while not rospy.is_shutdown():
        rospy.spin()
