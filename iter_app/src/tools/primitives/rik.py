'''
Primitives Relaxed-IK
Author: Curt Henrichs
Date: 5-2-19

Task primitives for runner implemented in Relaxed-IK.

While MoveIt provides many nice features to get started in developing primitives,
it is lacking in several aspects. Namely, repeatability of a task plan with same
joint paths. Relaxed-IK attempts to solve this problem by treating the pose goal
as an optimization problem where trade-offs in accuracy and joint travel are
weighed.

Using Wisconsin HCI's robot behavior package (developed for Authr), relaxed-ik was
successfuly integrated for all moveit supported primitives for the runner.

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
import time
import rospy

from enum import Enum
from std_msgs.msg import Header
from tools.pose_conversion import *
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Pose, Point, Quaternion
from tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive

from behavior_execution.planners.relaxedik import RelaxedIKPlanner


POSE_OFFSET = Pose(position=Point(),orientation=Quaternion())


planner = RelaxedIKPlanner(
    {"follow_joint_trajectory":"gripper_command"},
    {"follow_joint_trajectory":[
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint']},
    pose_offsets={"follow_joint_trajectory": POSE_OFFSET})


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    GET_POSE = 'get_pose'


class Grasp(Primitive):

    def __init__(self, effort=1):
        self._plan = planner.plan_gripper_state('follow_joint_trajectory',[effort])

    def operate(self):
        return planner.execute('follow_joint_trajectory',self._plan)

class Release(Primitive):

    def __init__(self, effort=1):
        self._plan = planner.plan_gripper_state('follow_joint_trajectory',[effort])

    def operate(self):
        return planner.execute('follow_joint_trajectory',self._plan)

class Move(Primitive):

    def __init__(self, position, orientation):
        # Convert from dictionary to Pose
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        plan = planner.plan_ee_pose('follow_joint_trajectory',self._pose)
        return planner.execute('follow_joint_trajectory',plan)

class GetPose(ReturnablePrimitive):

    def operate(self):
        return True, planner.get_ee_pose('follow_joint_trajectory')


class RelaxedIKBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(RelaxedIKBehaviorPrimitives,self).__init__(parent)

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
