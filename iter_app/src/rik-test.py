#!/usr/bin/env python

import rospy

from relaxed_ik.msg import EEPoseGoals, JointAngles

rospy.init_node('runner', anonymous=True)


def _subscriber(angleMsg):
    print(angleMsg)

eeGoal = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
jointAngles = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, _subscriber)

if __name__ == "__main__":

    while not rospy.is_shutdown():
        rospy.spin()
