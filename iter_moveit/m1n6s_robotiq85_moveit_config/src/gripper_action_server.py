import rospy
import actionlib
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from control_msgs.action import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

class GripperActionServer:

    def __init__(self, name):
        self._action_server = actionlib.SimpleActionServer('/gripper/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self._execute, False)
        self._action_server.start()

    def _execute(self, goal):
        print '\n\n\n\n', goal, '\n\n\n\n'
        

if __name__ == "__main__":
    rospy.init_node('gripper_action_server')
    server = GripperActionServer()
    rospy.spin()
