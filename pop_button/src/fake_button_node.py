#!/usr/bin/env python

'''
Fake Button Node
Author: Curt Henrichs
Date: 6-27-19

Provides the same ROS interface as the actual Pop button but uses the keyboard
instead. This should be used if the actual button is not available on the
network or for debugging.

Topics Published:

 - /button/pressed
    "Sends one pulse of True when button is first pressed"
    Type = std_msgs/bool

'''

import rospy
from std_msgs.msg import Bool


pressed_pub = rospy.Publisher('/button/pressed',Bool,queue_size=10)


if __name__ == "__main__":
    rospy.init_node("fake_button_node")

    while not rospy.is_shutdown():
        str = raw_input('Press enter button to trigger Fake Button Node')
        pressed_pub.publish(Bool(True))
        rospy.sleep(0.05)
