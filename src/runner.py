#!/usr/bin/env python

import sys
import time
import json
import rospy
import threading
import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('runner', anonymous=True)
import primitives as bp
import environment as env

from iter.msg import NeglectTime
from std_msgs.msg import Float32, String
from iter.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse
from time_mode_enum import TimeModeEnum


button_state = False
def button_callback():
    global button_state

    str = raw_input('Press enter button to stop wait')
    button_state = True
    return True #TODO write this for real

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

    def __init__(self,start_timing_callback,stop_timing_callback):
        self.time_mode = TimeModeEnum.REPLAY
        self.start_timing_callback = start_timing_callback
        self.stop_timing_callback = stop_timing_callback

        self.task_input_srv = rospy.Service('/runner/task_input', Task, self.run_task)
        self.mode_set_srv = rospy.Service('/runner/set/mode', ModeSet, self.mode_update)
        self.mode_get_srv = rospy.Service('/runner/get/mode', ModeGet, self.mode_get)

    def run_task(self, json_string):

        data = json.loads(json_string.task_json)

        if 'environment' in data.keys():
            env.generate_dynamic_environment(data['environment'])

        primitives = [bp.instantiate_from_dict(obj,button_callback) for obj in data['task']]
        neglect_time_list = generate_neglect_time_list(data)

        operate_status = True

        if self.time_mode == TimeModeEnum.REPLAY:
            self.start_timing_callback(neglect_time_list)

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

            if not operate_status:
                break

        if self.time_mode == TimeModeEnum.REPLAY:
            self.stop_timing_callback()

        if 'environment' in data.keys():
            env.clear_dynamic_environment()

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


class RadSignal:

    SIGNAL_PUBLISH_TIME_STEP = 0.01

    def __init__(self):
        self.pub_signal = rospy.Publisher('/rad/signal', Float32, queue_size=10)
        self.pub_neglect_time = rospy.Publisher('/rad/neglect_time', NeglectTime, queue_size=10)
        self.pub_interaction_time = rospy.Publisher('rad/interaction_time', Float32, queue_size=10)

        self._thread = None
        self._thread_alive = False
        self._neglect_time_list = []

    def start_timing(self,neglect_time_list):
        if self._thread_alive == False:
            self._neglect_time_list = neglect_time_list

            self._thread = threading.Thread(target=self._thread_fnt)
            self._thread_alive = True
            self._thread.start()
        else:
            raise Exception('Already running a timing loop')

    def stop_timing(self):
        self._thread_alive = False
        if self._thread != None:
            self._thread.join(1)
            self._thread = None

    def _thread_fnt(self):
        global button_state

        try:
            neglect_time = NeglectTime()
            interaction_time = 0

            for t in self._neglect_time_list:

                if 'interaction' in t.keys() and t['interaction']:
                    # Wait for interaction event
                    interaction_time = 0
                    current = base = time.time()

                    while not button_state and self._thread_alive:

                        interaction_time = current - base
                        self.pub_interaction_time.publish(interaction_time)

                        time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                        current = time.time()

                    button_state = False
                else:
                    # Run through the prerecorded timing
                    neglect_time.current = t["time"]
                    neglect_time.initial = t["time"]

                    current = base = time.time()

                    while current < base + t["time"] and self._thread_alive:

                        neglect_time.current = t["time"] - (current - base)
                        self.pub_neglect_time.publish(neglect_time)

                        time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                        current = time.time()

                if not self._thread_alive:
                    break

        except Exception, e:
            print e


if __name__ == '__main__':
    time.sleep(10) # wait for everything to setup first

    env.generate_static_environment()

    rad = RadSignal()
    runner = Runner(rad.start_timing,rad.stop_timing)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
        rad.stop_timing()
    except:
        rad.stop_timing()
