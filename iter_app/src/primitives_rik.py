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
import time
import rospy

from enum import Enum
from std_msgs.msg import Header
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    WAIT = 'wait'
    LOGGER = 'logger'
    CONNECT_OBJECT_TO_ROBOT = 'connect_object'
    DISCONNECT_OBJECT_FROM_ROBOT = 'disconnect_object'

class ConditionEnum(Enum):
    TIME = 'time'
    BUTTON = 'button'


class Primitive:
    __metaclass__ = ABCMeta

    @abstractmethod
    def operate(self):
        raise NotImplementedError('Primitive is Abstract')


class DebugLogger(Primitive):

    def __init__(self, msg):
        self._msg = msg

    def operate(self):
        print 'Logger Message: ', self._msg
        return True

class ConnectObjectToRobot(Primitive):

    def __init__(self,object_name):
        self.box_name = object_name

    def operate(self):
        # Note: Not supported by RIK implementation
        return True

class DisconnectObjectFromRobot(Primitive):

    def __init__(self,object_name):
        self.box_name = object_name

    def operate(self):
        # Note: Not supported by RIK implementation
        return True

class Grasp(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):
        # TODO
        return True

class Release(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):
        # TODO
        return True

class Move(Primitive):

    def __init__(self, position, orientation):
        # Convert from dictionary to Pose
        self._pose = Pose()
        self._pose.position.x = position['x']
        self._pose.position.y = position['y']
        self._pose.position.z = position['z']

        if 'w' not in orientation:
            (x,y,z,w)= tf.transformations.quaternion_from_euler(
                orientation['x'],
                orientation['y'],
                orientation['z'])
            self._pose.orientation.x = x
            self._pose.orientation.y = y
            self._pose.orientation.z = z
            self._pose.orientation.w = w
        else:
            self._pose.orientation.x = orientation['x']
            self._pose.orientation.y = orientation['y']
            self._pose.orientation.z = orientation['z']
            self._pose.orientation.w = orientation['w']

    def operate(self):
        # TODO
        return True

class Wait(Primitive):

    def __init__(self, condition, **kwargs):

        if kwargs is None:
            raise TypeError('Must supply additional arguements for condition')

        if 'timeout' in kwargs.keys():
            self._timeout = kwargs['timeout']
        else:
            self._timeout = None

        if condition == ConditionEnum.TIME.value:
            if not 'value' in kwargs.keys():
                raise TypeError('Must supply value arguement for time condition')

            self._cb = self._time_callback
            self._value = kwargs['value']

        elif condition == ConditionEnum.BUTTON.value:
            if not 'button_cb' in kwargs.keys() or kwargs['button_cb'] is None:
                raise ValueError('Button requires callback to be supplied')

            self._cb = kwargs['button_cb']
            self._value = True

        else:
            raise ValueError('Condition provided is unsupported')

        self.condition = condition

    def _time_callback(self):
        diff = time.time() - self.initial
        return diff >= self._value

    def operate(self):

        timeout = None
        if self._timeout != None:
            timeout = time.time() + self._timeout

        self.initial = time.time()

        ret_val = True
        while self._cb() != True:
            if timeout != None and time.time() >= timeout:
                ret_val = False
                break
            time.sleep(0.01)

        return ret_val


def instantiate_from_dict(obj, button_callback):

    name = obj['name']
    if name == PrimitiveEnum.GRASP.value:
        return Grasp(obj['effort'])
    elif name == PrimitiveEnum.RELEASE.value:
        return Release(obj['effort'])
    elif name == PrimitiveEnum.MOVE.value:
        return Move(obj['position'],obj['orientation'])
    elif name == PrimitiveEnum.WAIT.value:
        return Wait(button_cb=button_callback, **obj)
    elif name == PrimitiveEnum.LOGGER.value:
        return DebugLogger(obj['msg'])
    elif name == PrimitiveEnum.CONNECT_OBJECT_TO_ROBOT.value:
        return ConnectObjectToRobot(obj['object_name'])
    elif name == PrimitiveEnum.DISCONNECT_OBJECT_FROM_ROBOT.value:
        return DisconnectObjectFromRobot(obj['object_name'])
    else:
        raise Exception('Invalid behavior primitive supplied')
