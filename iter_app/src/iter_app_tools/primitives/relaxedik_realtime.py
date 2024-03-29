'''
Primitives Relaxed-IK (Realtime)
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
from iter_app_tools.pose_conversion import *
from geometry_msgs.msg import Pose, Point, Quaternion
from iter_app_tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive

from behavior_execution.planners.realtime_relaxedik import RealTimeRelaxedIKPlanner
from behavior_execution.planners.gripper_command import GripperCommandPlanner


pathToRikSrc = rospy.get_param('path_to_relaxed_ik_src')
infoFileName = rospy.get_param('info_file_name')
relaxedikPlanner = RealTimeRelaxedIKPlanner(pathToRikSrc,infoFileName,execute_timestep=0.01,plan_pose_step=0.005)
gripperPlanner = GripperCommandPlanner('gripper_command')


def initialize_robot():
    relaxedikPlanner.initialize()


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    GET_POSE = 'get_pose'


class Grasp(Primitive):

    def __init__(self, effort=1):
        self._plan = gripperPlanner.plan_gripper_state(effort)

    def operate(self):
        relaxedikPlanner.real_time_pause_output(True)
        status = gripperPlanner.execute_gripper_state(self._plan)
        relaxedikPlanner.real_time_pause_output(False)
        return status

class Release(Primitive):

    def __init__(self, effort=0):
        self._plan = gripperPlanner.plan_gripper_state(effort)

    def operate(self):
        relaxedikPlanner.real_time_pause_output(True)
        status = gripperPlanner.execute_gripper_state(self._plan)
        relaxedikPlanner.real_time_pause_output(False)
        return status

class Move(Primitive):

    def __init__(self, position, orientation, options={}, **kwargs):
        # Convert from dictionary to Pose
        self._options = options
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        plan = relaxedikPlanner.plan_ee_pose(self._pose, **self._options)
        status = relaxedikPlanner.execute_ee_pose(plan)
        return status

class GetPose(ReturnablePrimitive):

    def operate(self):
        return True, relaxedikPlanner.get_ee_pose()


class RelaxedIKRealTimeBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(RelaxedIKRealTimeBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, **kwargs):

        name = dct['name']
        if name == PrimitiveEnum.GRASP.value:
            return Grasp(dct['effort'])
        elif name == PrimitiveEnum.RELEASE.value:
            return Release(dct['effort'])
        elif name == PrimitiveEnum.MOVE.value:
            return Move(**dct)
        elif name == PrimitiveEnum.GET_POSE.value:
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
