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
    - /button/state (TODO ifttt connection)
rrikik
Parameters required:
    - use_rik
    - ~start_delay_time

'''


import sys
import time
import json
import rospy

from tools.pose_conversion import *
from geometry_msgs.msg import Vector3
from iter_app.msg import EnvironmentObject
from std_msgs.msg import String, Int32, Bool
from tools.time_mode_enum import TimeModeEnum
from tools.environment_client import EnvironmentClient
from iter_app.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse
from tools.primitives.abstract import Primitive, ReturnablePrimitive

rospy.init_node('runner')
envClient = EnvironmentClient()

from tools.primitives.default import DefaultBehaviorPrimitives

use_rik = rospy.get_param('use_rik',False)
if use_rik:
    from tools.primitives.relaxedik import RelaxedIKBehaviorPrimitives as PhysicalBehaviorPrimitives
else:
    from tools.primitives.moveit import MoveItBehaviorPrimitives as PhysicalBehaviorPrimitives

from tools.primitives.environment_aware import EnvironmentAwareBehaviorPrimitives

bp = EnvironmentAwareBehaviorPrimitives(envClient=envClient,
     parent=PhysicalBehaviorPrimitives(
     parent=DefaultBehaviorPrimitives()))


DEFAULT_NODE_START_DELAY_TIME = 10


class Runner:

    def __init__(self):
        self.time_mode = TimeModeEnum.REPLAY

        self.task_input_srv = rospy.Service('/runner/task_input', Task, self.run_task)
        self.mode_set_srv = rospy.Service('/runner/set/mode', ModeSet, self.mode_update)
        self.mode_get_srv = rospy.Service('/runner/get/mode', ModeGet, self.mode_get)

        self.time_start_topic = rospy.Publisher('time_node/start', String, queue_size=10)
        self.time_stop_topic = rospy.Publisher('time_node/stop', Bool, queue_size=10)
        self.time_sync_topic = rospy.Publisher('time_node/sync', Int32, queue_size=10)

    def run_task(self, json_string):
        global envClient, bp

        # data retrieved from interface
        try:
            data = json.loads(json_string.task_json)
        except:
            return TaskResponse(False,'{}')

        # if environment requested, load it
        if 'environment' in data.keys():
            objects = [self._env_obj_pkg(dct) for dct in data['environment']]
            resp = envClient.generate_task_objects(objects)
            if not resp.status:
                return TaskResponse(False,'{}')

        print 'Instantiating'
        primitives = [bp.instantiate_from_dict(obj,button_callback=self._button_callback) for obj in data['task']]
        neglect_time_list = self._generate_neglect_time_list(data)

        # provide timing information to timing node
        if self.time_mode == TimeModeEnum.REPLAY:
            self.time_start_topic.publish(json.dumps(neglect_time_list))

        # iterate over all primitives
        print 'Running'
        operate_status = True
        for i in range(0,len(primitives)):
            print type(primitives[i]).__name__

            if self.time_mode == TimeModeEnum.CAPTURE:
                if type(primitives[i]) is bp.Wait and primitives[i].condition == bp.ConditionEnum.BUTTON.value:
                    data['task'][i]['rad'] = {'is_interaction': True}

                    if isinstance(ReturnablePrimitive,primitives[i]):
                        operate_status, result = primitives[i].operate()
                    else:
                        operate_status = primitives[i].operate()

                else:
                    start = time.time()

                    if isinstance(ReturnablePrimitive,primitives[i]):
                        operate_status, result = primitives[i].operate()
                    else:
                        operate_status = primitives[i].operate()

                    stop = time.time()

                    data['task'][i]['rad'] = {
                        'neglect_time': stop - start,
                        'is_interaction': False
                    }
            else:

                if isinstance(primitives[i],ReturnablePrimitive):
                    operate_status, result = primitives[i].operate()
                else:
                    operate_status = primitives[i].operate()

                self.time_sync_topic.publish(i)

            if not operate_status:
                break

        # stop timing
        if self.time_mode == TimeModeEnum.REPLAY:
            self.time_stop_topic.publish(True)

        # clear environment resources
        if 'environment' in data.keys():
            clear_status = envClient.clear_task_objects([],True).status
            operate_status = operate_status and clear_status

        # send results back to interface
        if self.time_mode == TimeModeEnum.CAPTURE:
            return TaskResponse(operate_status,json.dumps(data))
        else:
            return TaskResponse(operate_status,'{}')

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
                    time_list.append({"time": temp_neglect_time})
                    temp_neglect_time = 0
                    time_list.append({"interaction": True})
                elif 'neglect_time' in obj['rad']: # increment by amount
                    temp_neglect_time += obj['rad']['neglect_time']

        time_list.append({"time": temp_neglect_time})

        return time_list

    def _env_obj_pkg(self,obj_dct):
        return EnvironmentObject(representation=EnvironmentObject.REPRESENTATION_BOX,
                                 id=obj_dct['name'],
                                 size=Vector3(x=obj_dct['size']['x'],
                                              y=obj_dct['size']['y'],
                                              z=obj_dct['size']['z']),
                                 pose=pose_dct_to_msg(obj_dct))

    def _button_callback(self):
        #TODO write this for real
        str = raw_input('Press enter button to stop wait')
        button_state = True
        return button_state


if __name__ == '__main__':

    start_delay_time = rospy.get_param('~start_delay_time',DEFAULT_NODE_START_DELAY_TIME)
    rospy.sleep(start_delay_time)
    print "\n\n\n Runner is Ready\n\n\n"

    runner = Runner()

    while not rospy.is_shutdown():
        rospy.spin()
