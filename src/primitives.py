
"""
{
    'task': [
        {
            'name': <string ['wait, move, grasp, release']>
            ... <params>
        }
    ]
}
"""

import tf
import sys
import rospy
import time

from abc import ABCMeta
from enum import Enum

import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)

##TODO figure out gripper for grasp primitives

class PrimitiveEnum(Enum):
    GRASP = 'grasp',
    RELEASE = 'release',
    MOVE = 'move',
    WAIT = 'wait'


class ConditionEnum(Enum):
    TIME = 'time'
    BUTTON = 'button'


class Primitive:
    __metaclass__ = ABCMeta

    @abstractmethod
    def operate(self):
        raise NotImplementedError('Primitive is Abstract')


class Grasp(Primitive):

    def __init__(self, effort=1):
        self.robot_commander = moveit_commander.RobotCommander()
        self.scene_interface = moveit_commander.PlanningSceneInterface()


    def operate(self):
        pass


class Release(Primitive):

    def __init__(self):
        self.robot_commander = moveit_commander.RobotCommander()
        self.scene_interface = moveit_commander.PlanningSceneInterface()

    def operate(self):
        pass


class Move(Primitive):

    def __init__(self, position, orientation):
        self.move_group_commander = moveit_commander.MoveGroupCommander('manipulator')

        # Convert to Pose
        self._pose = Pose()
        self._pose.position = position
        self._pose.orientation = tf.transformations.quaternion_from_euler(
            orientation['x'],
            orientation['y'],
            orientation['z'])

    def operate(self):
        self.move_group_commander.clear_pose_targets()
        self.move_group_commander.set_pose_target(self._pose)
        self.move_group_commander.go(wait=True)


class Wait(Primitive):

    def __init__(self, condition, **kwargs):

        if kwargs is None:
            raise TypeError('Must supply additional arguements for condition')

        if 'timeout' in kwargs.keys():
            self._timeout = kwargs['timeout']
        else:
            self._timeout = None

        if condition == ConditionEnum.TIME:
            if not 'value' in kwargs.keys():
                raise TypeError('Must supply value arguement for time condition')

            self._cb = self._time_callback
            self._value = kwargs['value']

        elif condition == ConditionEnum.BUTTON:
            if not 'button_cb' in kwargs.keys() or kwargs['button_cb'] is None:
                raise ValueError('Button requires callback to be supplied')

            self._cb = button_cb
            self._value = True

        else:
            raise ValueError('Condition provided is unsupported')

    def _time_callback(self):
        return time.time() >= self._value

    def operate(self):

        timeout = None
        if self._timeout != None:
            timeout = time.time() + self._timeout

        ret_val = True
        while self._cb() != self._value:
            if timeout != None and time.time() >= timeout:
                ret_val = False
                break
            time.sleep(0.01)

        return ret_val

class PrimitiveEnum(Enum):
    GRASP = 'grasp',
    RELEASE = 'release',
    MOVE = 'move',
    WAIT = 'wait'


def instantiate_from_dict(obj):

    name = obj['name']
    if name == PrimitiveEnum.GRASP:
        pass
    elif name == PrimitiveEnum.RELEASE:
        pass
    elif name == PrimitiveEnum.MOVE:
        pass
    elif name == PrimitiveEnum.WAIT:
        pass
    else:
        raise Exception('Invalid behavior primitive supplied')
