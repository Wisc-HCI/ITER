#!/usr/bin/env python

'''
Initialize Joints
Author: Curt Henrichs
Date: 8-9-19

Initilizes the joints for the robot.
'''

import rospy
rospy.init_node('initialize_joints')

from std_msgs.msg import String


DEFAULT_URSCRIPT_TOPIC = 'ur_driver/URScript'
DEFAULT_ROBOT = 'ur3e'
DEFAULT_WAIT_TIME = 10


robot = rospy.get_param('~robot',DEFAULT_ROBOT)
wait_time = rospy.get_param('~wait_time',DEFAULT_WAIT_TIME)
ur_script_topic = rospy.get_param('~urscript_topic',DEFAULT_URSCRIPT_TOPIC)
joint_pub = rospy.Publisher(ur_script_topic,String,queue_size=5)


if __name__ == "__main__":

    rospy.sleep(wait_time)

    if robot == 'ur3e':
        js = [-1.039398495350973, -1.2305773061564942, -1.8399834632873535, -1.643841405908102, 1.5702834129333496, -1.037588898335592]
    elif robot == 'ur5':
        js = [ 0, 0, 0, 0, 0, 0 ]
    else:
        raise Exception('Invalid robot provided')

    cmd = 'movej([{0},{1},{2},{3},{4},{5}])'.format(js[0],js[1],js[2],js[3],js[4],js[5])
    joint_pub.publish(String(cmd))
