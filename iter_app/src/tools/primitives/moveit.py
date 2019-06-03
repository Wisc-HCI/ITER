'''
Primitives Moveit
Author: Curt Henrichs
Date: 5-2-19

Task primitives for runner implemented in MoveIt.

MoveIt provides a robust and comprehensive interface to control a robotic arm
and thus is a natural choice to develop the first iteration of ITER's primitive
library.

Primitve List:
    - grasp
            "Move the gripper to state specfied as target"
    - release
            "move the gripper, (identical to grasp)"
    - move
            "Commands robot to move end-effector to provided pose goal"
    - get_pose
            ""
'''

import tf
import sys
import time
import rospy
import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

from enum import Enum
from std_msgs.msg import Header
from tools.pose_conversion import *
from abc import ABCMeta, abstractmethod
from tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive


ARM_MOVE_GROUP = rospy.get_param("arm_move_group")
GRIPPER_MOVE_GROUP = rospy.get_param("gripper_move_group")
GRIPPER_TYPE = rospy.get_param("gripper_type")


arm_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
gripper_group_commander = moveit_commander.MoveGroupCommander(GRIPPER_MOVE_GROUP)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    GET_POSE = 'get_pose'


class Grasp(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):

        arm_group_commander.clear_pose_targets()

        if GRIPPER_TYPE == 'mico-3':
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_2':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_3':self._effort})
        elif GRIPPER_TYPE == 'mico-2':
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_2':self._effort})
        else: # Robotiq 85
            joint = gripper_group_commander.get_joints()[0]
            gripper_group_commander.set_joint_value_target({joint:self._effort})

        gripper_group_commander.go(wait=True)
        gripper_group_commander.stop()
        return True

class Release(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):

        arm_group_commander.clear_pose_targets()

        if GRIPPER_TYPE == 'mico-3':
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_2':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_3':self._effort})
        elif GRIPPER_TYPE == 'mico-2':
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_2':self._effort})
        else: # Robotiq 85
            joint = gripper_group_commander.get_joints()[0]
            gripper_group_commander.set_joint_value_target({joint:self._effort})

        gripper_group_commander.go(wait=True)
        gripper_group_commander.stop()
        return True

class Move(Primitive):

    def __init__(self, position, orientation):
        # Convert from dictionary to Pose
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        global arm_group_commander

        arm_group_commander.clear_pose_targets()
        arm_group_commander.set_pose_target(self._pose)
        retVal = arm_group_commander.go(wait=True)
        arm_group_commander.stop()
        return retVal

class GetPose(ReturnablePrimitive):

    def operate(self):
        return True, arm_group_commander.get_current_pose().pose


class MoveItBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(MoveItBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, **kwargs):

        name = dct['name']
        if name == PrimitiveEnum.GRASP.value:
            return Grasp(dct['effort'])
        elif name == PrimitiveEnum.RELEASE.value:
            return Release(dct['effort'])
        elif name == PrimitiveEnum.MOVE.value:
            return Move(dct['position'],dct['orientation'])
        elif name = PrimitiveEnum.GET_POSE.value:
            return GetPose()
        elif self.parent != None:
            return self.parent.instantiate_from_dict(dct,**kwargs)
        else:
            raise Exception('Invalid behavior primitive supplied')

    def lookup(self, primitive_type):
        if primitive_type == PrimitiveEnum.GRASP.value:
            return Grasp
        elif primitive_type == PrimitiveEnum.RELEASE.value:
            return Release
        elif primitive_type == PrimitiveEnum.MOVE.value:
            return Move
        elif primitive_type == PrimitiveEnum.GET_POSE.value:
            return GetPose
        elif self.parent != None:
            return self.parent.lookup(primitive_type)
        else:
            raise Exception('Invalid behavior primitive supplied')
