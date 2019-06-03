'''

'''

from abc import ABCMeta, abstractmethod


class Primitive:
    __metaclass__ = ABCMeta

    @abstractmethod
    def operate(self): # Return Boolean status
        raise NotImplementedError('Primitive is Abstract')

class ReturnablePrimitive(Primitive):
    __metaclass__ = ABCMeta

    @abstractmethod
    def operate(self): # Return Boolean status, Return result
        raise NotImplementedError('Primitive is Abstract')


class AbstractBehaviorPrimitives:
    __metaclass__ = ABCMeta

    def __init__(self,parent=None):
        self.parent = parent

    @abstractmethod
    def instantiate_from_dict(self, dct, **kwargs):
        pass

    @abstractmethod
    def lookup(self, primitive_type):
        pass
