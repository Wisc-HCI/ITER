#!/usr/bin/env python

import time
import json
import rospy
import threading
import primitives as bp

from enum import Enum
from iter.msg import NeglectTime
from std_msgs.msg import Float32, String
from iter.srv import Task, TaskResponse, Mode, ModeResponse


class TimeModeEnum(Enum):
    REPLAY = 'replay'
    CAPTURE = 'capture'

    @classmethod
    def from_str(cls, mode):
        if mode == cls.REPLAY.value:
            return cls.REPLAY
        elif mode == cls.CAPTURE.value:
            return cls.CAPTURE
        else:
            return None


class Runner:

    def __init__(self):
        self.time_mode = TimeModeEnum.REPLAY

        self.task_input_srv = rospy.Service('/runner/task_input', Task, self.run_task)
        self.mode_srv = rospy.Service('/runner/mode', Mode, self.mode_update)

    def run_task(self, json_string):

        data = json.loads(json_string.task_json)
        primitives = [bp.instantiate_from_dict(obj) for obj in data['task']]

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
                        'negelect_time': stop - start,
                        'is_interaction': False
                    }
            else:
                operate_status = primitives[i].operate()

            if not operate_status:
                break

        if self.time_mode == TimeModeEnum.CAPTURE:
            return TaskResponse(operate_status,json.dumps(data))
        else:
            return TaskResponse(operate_status,'')

    def mode_update(self, data):
        mode = TimeModeEnum.from_str(data.mode)
        if mode == None:
            raise Exception('Invalid mode supplied')

        self.time_mode = mode
        return ModeResponse()


class RadSignal:

    def __init__(self):

        self.rad_signal = 1
        self.neglect_time = NeglectTime()
        self.neglect_time.current = 65
        self.neglect_time.initial = 65
        self.interaction_time = 60

        self.pub_signal = rospy.Publisher('/rad/signal', Float32, queue_size=10)
        self.pub_neglect_time = rospy.Publisher('/rad/neglect_time', NeglectTime, queue_size=10)
        self.pub_interaction_time = rospy.Publisher('rad/interaction_time', Float32, queue_size=10)

    def start(self):
        self._thread = threading.Thread(target=self._thread_fnt)
        self._thread_alive = True
        self._thread.start()

    def stop(self):
        self._thread_alive = False
        self._thread.join(1)

    def _compute(self):
        # RAD = IT / (IT + NT)
        self.rad_signal = self.interaction_time * 1.0 / (self.interaction_time + self.neglect_time.current)

    def _publish(self):
        self.pub_signal.publish(self.rad_signal)
        self.pub_neglect_time.publish(self.neglect_time)
        self.pub_interaction_time.publish(self.interaction_time)

    def _thread_fnt(self):
        while self._thread_alive:
            self._compute()
            self._publish()
            time.sleep(1)


if __name__ == '__main__':

    runner = Runner()
    rad = RadSignal()

    rospy.init_node('runner', anonymous=True)
    rad.start()

    while not rospy.is_shutdown():
        rospy.spin()

    rad.stop()
