'''
    - wait
            "Provides either interaction button or time based delay"
    - debug-logger
            "Provides print functionality while debugging plan"
'''

import time

from enum import Enum
from abc import ABCMeta, abstractmethod


class PrimitiveEnum(Enum):
    WAIT = 'wait'
    LOGGER = 'logger'

class ButtonConditionEnum(Enum):
    TIME = 'time'
    BUTTON = 'button'


class Primitive:
    __metaclass__ = ABCMeta

    @abstractmethod
    def operate(self):
        raise NotImplementedError('Primitive is Abstract')


class Wait(Primitive):

    def __init__(self, condition, **kwargs):

        if kwargs is None:
            raise TypeError('Must supply additional arguements for condition')

        if 'timeout' in kwargs.keys():
            self._timeout = kwargs['timeout']
        else:
            self._timeout = None

        if condition == ButtonConditionEnum.TIME.value:
            if not 'value' in kwargs.keys():
                raise TypeError('Must supply value arguement for time condition')

            self._cb = self._time_callback
            self._value = kwargs['value']

        elif condition == ButtonConditionEnum.BUTTON.value:
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

class DebugLogger(Primitive):

    def __init__(self, msg):
        self._msg = msg

    def operate(self):
        print 'Logger Message: ', self._msg
        return True


class AbstractBehaviorPrimitives:
    __metaclass__ = ABCMeta

    def __init__(self, envClient)
        self._envClient = envClient

    def instantiate_from_dict(self, obj, button_callback):

        name = obj['name']
        if name == PrimitiveEnum.WAIT.value:
            return Wait(button_cb=button_callback, **obj)
        elif name == PrimitiveEnum.LOGGER.value:
            return DebugLogger(obj['msg'])
        elif name == PrimitiveEnum.CALIBRATE_ROBOT_TO_CAMERA.value:
            return CalibrateRobotToCamera()
        else:
            raise Exception('Invalid behavior primitive supplied')
