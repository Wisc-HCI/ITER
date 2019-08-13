'''
Primitives UR Direct
Author: Curt Henrichs
Date: 8-9-19

Task primitives for runner implemented using direct UR control

This interface circumvents IK solvers on ROS machine, instead utilizing internal
UR controller's solver.
'''

import rospy

from enum import Enum
from std_msgs.msg import Header
from iter_app_tools.pose_conversion import *
from geometry_msgs.msg import Pose, Point, Quaternion
from iter_app_tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive

from behavior_execution.planners.ur_direct import URDirectPlanner
from behavior_execution.planners.gripper_command import GripperCommandPlanner


urDirectPlanner = URDirectPlanner([ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ])
gripperPlanner = GripperCommandPlanner('gripper_command')


def initialize_robot():
    pass


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    GET_POSE = 'get_pose'


class Grasp(Primitive):

    def __init__(self, effort=1):
        self._plan = gripperPlanner.plan_gripper_state(effort)

    def operate(self):
        return gripperPlanner.execute_gripper_state(self._plan)

class Release(Primitive):

    def __init__(self, effort=0):
        self._plan = gripperPlanner.plan_gripper_state(effort)

    def operate(self):
        return gripperPlanner.execute_gripper_state(self._plan)

class Move(Primitive):

    def __init__(self, position, orientation, options, **kwargs):
        # Convert from dictionary to Pose
        self._options = options
        if not 'motion_type' in self._options.keys():
            self.options['motion_type'] = 'joint'
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        return urDirectPlanner.set_ee_pose(self._pose, **self._options)

class GetPose(ReturnablePrimitive):

    def operate(self):
        return True, urDirectPlanner.get_ee_pose()


class URDirectBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(URDirectBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, **kwargs):

        name = dct['name']
        if name == PrimitiveEnum.GRASP.value:
            return Grasp(dct['effort'])
        elif name == PrimitiveEnum.RELEASE.value:
            return Release(dct['effort'])
        elif name == PrimitiveEnum.MOVE.value:
            return Move(**kwargs)
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
