#!/usr/bin/env python

import json
import rospy
from iter.msg import Task, TaskResponse
from std_msgs.msg import Float32, String
from experiment_therblig_task_runner.msg import NeglectTime

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

def run_task(data):
    d = json.loads(data.task)

    for obj in d['task']:
        primitive = instantiate_from_dict(obj)
        primitive.operate()

    return TaskResponse(True)

def compute_rad():
    # compute signals This is just a temporary version
    if neglect_time.current > 0:
        neglect_time.current -= 1
    else:
        neglect_time.current = neglect_time.initial

    rad_signal = interaction_time * 1.0 / (interaction_time + neglect_time.current)

def publish_rad():
    pub_signal.publish(rad_signal)
    pub_neglect_time.publish(neglect_time)
    pub_interaction_time.publish(interaction_time)


if __name__ == '__main__':
    try:

        # Define Published Topics
        pub_signal = rospy.Publisher('/rad/signal', Float32, queue_size=10)
        pub_neglect_time = rospy.Publisher('/rad/neglect_time', NeglectTime, queue_size=10)
        pub_interaction_time = rospy.Publisher('rad/interaction_time', Float32, queue_size=10)

        # Define Services
        task_input_srv = rospy.Service('task_input', Task, run_task)

        # Run Node
        rospy.init_node('runner', anonymous=True)
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
