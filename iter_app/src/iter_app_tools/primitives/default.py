'''
    - wait
            "Provides either interaction button or time based delay"
    - logger
            "Provides print functionality while debugging plan"
'''

import os
import time
import rospy
import importlib

from enum import Enum
from std_msgs.msg import String
from iter_app_tools.primitives.abstract import Primitive, ReturnablePrimitive, AbstractBehaviorPrimitives

class PrimitiveEnum(Enum):
    WAIT = 'wait'
    LOGGER = 'logger'
    PROMPT = 'prompt'
    PUBLISH = 'publish'
    PUBLISH_STRING_FROM_FILE = 'publish_string_from_file'

class ButtonConditionEnum(Enum):
    TIME = 'time'
    BUTTON = 'button'


publishers = {}


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

class Prompt(ReturnablePrimitive):

    def __init__(self, msg):
        self._msg = msg

    def operate(self):
        try:
            response = raw_input(self._msg)
            return True, response
        except:
            return False, ''

class Publish(Primitive):

    def __init__(self, topic, message_module, message_type, message_params, **kwargs):
        # Note: this does not work for all message types (specifically nested messages)
        module = importlib.import_module('{}.msg'.format(message_module))
        assert hasattr(module, message_type), "class {} is not in {}".format(message_type, message_module)
        message = getattr(module,message_type)
        self._msg = message(**message_params)

        self._topic = topic
        if not topic in publishers.keys():
            publishers[self._topic] = rospy.Publisher(topic,message,queue_size=1)

    def operate(self):
        publishers[self._topic].publish(self._msg)
        return True

class PublishStringFromFile(Primitive):

    def __init__(self, topic, filepath, **kwargs):
        self._topic = topic
        if not topic in publishers.keys():
            publishers[self._topic] = rospy.Publisher(topic,String,queue_size=1)

        path = os.path.join(os.getcwd(),filepath)
        fin = open(path,'r')
        _str = fin.read().encode('utf-8')
        self._msg = String(_str)
        fin.close()

    def operate(self):
        print self._msg
        publishers[self._topic].publish(self._msg)
        return True


class DefaultBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self,parent=None):
        super(DefaultBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, button_callback, **kwargs):
        name = dct['name']
        if name == PrimitiveEnum.WAIT.value:
            return Wait(button_cb=button_callback, **dct)
        elif name == PrimitiveEnum.LOGGER.value:
            return DebugLogger(dct['msg'])
        elif name == PrimitiveEnum.PROMPT.value:
            return Prompt(dct['msg'])
        elif name == PrimitiveEnum.PUBLISH.value:
            return Publish(**dct)
        elif name == PrimitiveEnum.PUBLISH_STRING_FROM_FILE.value:
            return PublishStringFromFile(**dct)
        elif self.parent != None:
            if not 'button_callback' in kwargs.keys():
                kwargs['button_callback'] = button_callback
            return self.parent.instantiate_from_dict(dct,**kwargs)
        else:
            raise Exception('Invalid behavior primitive supplied')

    def lookup(self, primitive_type):
        if primitive_type == PrimitiveEnum.WAIT.value:
            return Wait
        elif primitive_type == PrimitiveEnum.LOGGER.value:
            return DebugLogger
        elif primitive_type == PrimitiveEnum.PROMPT.value:
            return Prompt
        elif primitive_type == PrimitiveEnum.PUBLISH.value:
            return Publish
        elif primitive_type == PrimitiveEnum.PUBLISH_STRING_FROM_FILE.value:
            return PublishStringFromFile
        elif self.parent != None:
            return self.parent.lookup(primitive_type)
        else:
            raise Exception('Invalid behavior primitive supplied')
