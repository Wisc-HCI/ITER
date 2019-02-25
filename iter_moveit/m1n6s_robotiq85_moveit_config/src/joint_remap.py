#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("joint_remapper")
jointPublisher = rospy.Publisher('joint_state', JointState, queue_size=15)

def callback(data):
    jointPublisher.publish(data)

jointSubscriber_gripper = rospy.Subscriber('/gripper/joint_states', JointState, callback)
jointSubscriber_arm = rospy.Subscriber('/m1n6s200_driver/out/joint_state', JointState, callback)

rospy.spin()
