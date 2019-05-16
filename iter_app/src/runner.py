#!/usr/bin/env python

## Note: Currently the environment is unsupported due to moveit having an issue
## with stack smashing and with relaxed IK there is no built in support.

import sys
import time
import json
import rospy

from time_mode_enum import TimeModeEnum
from std_msgs.msg import String, Int32, Bool
from iter_app.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse

rospy.init_node('runner')

use_rik = rospy.get_param('use_rik',False)
if use_rik:
    import primitives_rik as bp
else:
    import primitives_moveit as bp


def button_callback():
    #TODO write this for real
    str = raw_input('Press enter button to stop wait')
    button_state = True
    return button_state

def generate_neglect_time_list(task_dict):
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

        # data retrieved from interface
        data = json.loads(json_string.task_json)

        # if environment requested, load it
        #if 'environment' in data.keys():
        #    env.generate_dynamic_environment(data['environment'])

        print 'Instantiating'
        primitives = [bp.instantiate_from_dict(obj,button_callback) for obj in data['task']]
        neglect_time_list = generate_neglect_time_list(data)

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
                    operate_status = primitives[i].operate()
                else:
                    start = time.time()
                    operate_status = primitives[i].operate()
                    stop = time.time()

                    data['task'][i]['rad'] = {
                        'neglect_time': stop - start,
                        'is_interaction': False
                    }
            else:
                operate_status = primitives[i].operate()
                self.time_sync_topic.publish(i)

            if not operate_status:
                break

        # stop timing
        if self.time_mode == TimeModeEnum.REPLAY:
            self.time_stop_topic.publish(True)

        # clear environment resources
        #if 'environment' in data.keys():
        #    env.clear_dynamic_environment()

        # send results back to interface
        if self.time_mode == TimeModeEnum.CAPTURE:
            return TaskResponse(operate_status,json.dumps(data))
        else:
            return TaskResponse(operate_status,'')

    def mode_update(self, data):
        mode = TimeModeEnum.from_str(data.mode)
        if mode == None:
            raise Exception('Invalid mode supplied')

        self.time_mode = mode
        return ModeSetResponse()

    def mode_get(self, data):
        return ModeGetResponse(self.time_mode.value)


if __name__ == '__main__':
    rospy.sleep(10) # wait for everything to setup first
    print "\n\n\n Runner is Ready\n\n\n"
    runner = Runner()

    while not rospy.is_shutdown():
        rospy.spin()
