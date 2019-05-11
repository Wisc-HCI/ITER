#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('m1n6s_robotiq85_moveit_config')
import rospy
import actionlib

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult


LOWER_LIMIT = 0.0
UPPER_LIMIT = 0.085

TIME_THRESHOLD = 0.25
POSITON_THRESHOLD = 0.001


class GripperActionServer:

    def __init__(self):

        self._statSub = rospy.Subscriber('/gripper/stat', GripperStat, self._update_gripper_stat, queue_size=10)
        self._cmdPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        self._stat = GripperStat()

        self._action_server = actionlib.SimpleActionServer('gripper/gripper_command', GripperCommandAction, self._execute, False)
        self._action_server.start()

    def _update_gripper_stat(self, stat):
        self._stat = stat

    def _reached_goal(self, goal):
        return (self._stat.position <= (goal + POSITON_THRESHOLD) and self._stat.position >= (goal + POSITON_THRESHOLD))

    def _generate_result_msg(self, goal):
        result = GripperCommandResult()
        result.position = self._stat.position
        result.effort = 0
        result.stalled = False
        result.reached_goal = self._reached_goal(goal)
        return result

    def _generate_feedback_msg(self, goal):
        feedback = GripperCommandFeedback()
        feedback.position = self._stat.position
        feedback.effort = 0
        feedback.stalled = False
        feedback.reached_goal = self._reached_goal(goal)
        return feedback

    def _execute(self, goal):
        print '\n\n\n\n', goal, '\n\n\n\n'
        goalPosition = UPPER_LIMIT - goal.command.position / 10.0
        # ignore effort for now

        # check gripper is ready to accept commands
        if not self._stat.is_ready:
            print 'Gripper is not in ready state'
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # verify position is valid
        if goalPosition < LOWER_LIMIT or goalPosition > UPPER_LIMIT:
            print 'Position limits are exceeded'
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # send gripper command
        cmd = GripperCmd()
        cmd.position = goalPosition
        cmd.speed = 0.02
        cmd.force = 100.0
        self._cmdPub.publish(cmd)
        rospy.sleep(0.1)

        # move gripper
        preempted = False
        start_time = time.time()
        while self._stat.is_moving and (time.time() - start_time) < TIME_THRESHOLD:
            if self._action_server.is_preempt_requested():
                cmd = GripperCmd()
                cmd.stop = True
                self._cmdPub.publish(cmd)
                preempted = True
                break

            rospy.sleep(0.05)
            self._action_server.publish_feedback(self._generate_feedback_msg(goalPosition))

        # final result
        result = self._generate_result_msg(goalPosition)
        if preempted:
            print 'Preempted'
            self._action_server.set_preempted(result)
        elif result.reached_goal:
            print 'Reached goal state'
            self._action_server.set_succeeded(result)
        else:
            print 'Did not reach goal state'
            self._action_server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node('gripper_action_server')
    server = GripperActionServer()
    rospy.spin()
