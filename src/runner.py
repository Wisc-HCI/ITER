#!/usr/bin/env python

import time
import json
import rospy
import threading
import primitives as bp
from iter.srv import Task, TaskResponse, Mode, ModeResponse
from std_msgs.msg import Float32, String
from experiment_therblig_task_runner.msg import NeglectTime

from enum import Enum

class TimeModeEnum(Enum):
    REPLAY = 'replay'
    CAPTURE = 'capture'

rad_thread_alive = True

time_mode = TimeModeEnum.REPLAY.value


# RAD = IT / (IT + NT)
rad_signal = 1
neglect_time = NeglectTime()
neglect_time.current = 65
neglect_time.initial = 65
interaction_time = 60
countUp = False

"""
{
    'task': [
        {
            'name': <string ['wait, move, grasp, release']>
            ... <params>
        }
    ]
}
"""

def rad_thread():
    while rad_thread_alive:
        compute_rad()
        publish_rad()
        time.sleep(1)

def run_task(json_string):

    data = json.loads(json_string.task_json)
    primitives = [bp.instantiate_from_dict(obj) for obj in data['task']]

    for i in range(0,len(primitives)):
        print type(primitives[i]).__name__
        print data['task'][i]
        primitives[i].operate()

    return TaskResponse(True)

def compute_rad():
    rad_signal = interaction_time * 1.0 / (interaction_time + neglect_time.current)

def publish_rad():
    pub_signal.publish(rad_signal)
    pub_neglect_time.publish(neglect_time)
    pub_interaction_time.publish(interaction_time)

def mode_update():
    pass


if __name__ == '__main__':

    # Define Published Topics
    pub_signal = rospy.Publisher('/rad/signal', Float32, queue_size=10)
    pub_neglect_time = rospy.Publisher('/rad/neglect_time', NeglectTime, queue_size=10)
    pub_interaction_time = rospy.Publisher('rad/interaction_time', Float32, queue_size=10)

    # Define Services
    task_input_srv = rospy.Service('/iter/task_input', Task, run_task)
    mode_srv = rospy.Service('/iter/mode', Mode, mode_update)

    # Create RAD Thread
    rad_thread = threading.Thread(target=rad_thread)

    # Run Node
    rospy.init_node('runner', anonymous=True)
    rad_thread.start()
    while not rospy.is_shutdown():
        rospy.spin()

    rad_thread_alive = False
