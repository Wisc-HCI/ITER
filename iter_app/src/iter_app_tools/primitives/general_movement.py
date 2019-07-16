'''
'''

import tf
import rospy
import readchar

from enum import Enum
from iter_app_tools.pose_conversion import *
from iter_app_tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive


class PrimitiveEnum(Enum):
    JOG_POSE = 'jog_pose'


class JogPose(Primitive):
    '''
    Mapping:
        Position:
            w = +y
            s = -y
            a = -x
            d = +x
            q = +z
            z = -z
        Orientation RPY:
            j = -r
            l = +r
            i = +p
            k = -p
            u = +y
            m = -y
        General:
            c = quit
    '''

    def __init__(self, lookup, position_step, rotation_step):
        self._lookup = lookup
        self._position_step = position_step
        self._rotation_step = rotation_step

    def operate(self):

        # get current pose of EE
        get_pose = self._lookup('get_pose')()
        status, pose_msg = get_pose.operate()
        current_pose = pose_msg_to_dct(pose_msg)

        # start reading current until requested to quit
        running = status
        while running:

            # Read character
            key = readchar.readkey()
            if key == 'w':
                current_pose['position']['y'] += self._position_step
            elif key == 's':
                current_pose['position']['y'] -= self._position_step
            elif key == 'a':
                current_pose['position']['x'] -= self._position_step
            elif key == 'd':
                current_pose['position']['x'] += self._position_step
            elif key == 'q':
                current_pose['position']['z'] += self._position_step
            elif key == 'z':
                current_pose['position']['z'] -= self._position_step
            elif key == 'i':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[1] = rot[1] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'k':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[1] = rot[1] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'j':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[0] = rot[0] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'l':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[0] = rot[0] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'u':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[2] = rot[2] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'm':
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[2] = rot[2] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'c':
                running = False
            else:
                continue

            # Move to position
            move = self._lookup('move')(current_pose['position'],current_pose['orientation'])
            status = move.operate()
            if not status:
                running = False

        return status

    def _pack_quaternion(self,dct):
        return [dct['x'],dct['y'],dct['z'],dct['w']]


class GeneralMovementBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(GeneralMovementBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, **kwargs):
        name = dct['name']
        if name == PrimitiveEnum.JOG_POSE.value:
            return JogPose(self.lookup,dct['position_step'],dct['rotation_step'])
        elif self.parent != None:
            return self.parent.instantiate_from_dict(dct,**kwargs)
        else:
            raise Exception('Invalid behavior primitive supplied')

    def lookup(self, primitive_type):
        if primitive_type == PrimitiveEnum.JOG_POSE.value:
            return JogPose
        elif self.parent != None:
            return self.parent.lookup(primitive_type)
        else:
            raise Exception('Invalid behavior primitive supplied')
