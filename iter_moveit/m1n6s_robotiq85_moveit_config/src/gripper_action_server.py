#!/usr/bin/env python
import roslib
roslib.load_manifest('m1n6s_robotiq85_moveit_config')
import rospy
import actionlib

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

class GripperActionServer:

    def __init__(self):
        self._action_server = actionlib.SimpleActionServer('/gripper/follow_joint_trajectory', FollowJointTrajectoryAction, self._execute, False)
        self._action_server.start()

    def _execute(self, goal):
        print '\n\n\n\n', goal, '\n\n\n\n'
        self._action_server.set_succeeded()


if __name__ == "__main__":
    rospy.init_node('gripper_action_server')
    server = GripperActionServer()
    rospy.spin()
