'''
'''

import tf
import rospy
import readchar

from enum import Enum
from iter_app_tools.pose_conversion import *
from iter_app_tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive


class PrimitiveEnum(Enum):
    JOG = 'jog'


class Jog(Primitive):
    '''
    Mapping:
        Position XYZ:
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
        Grip:
            g = +j
            h = -j
        General:
            c = quit
    '''

    def __init__(self, lookup, position_step, rotation_step, grip_step):
        self._lookup = lookup
        self._position_step = position_step
        self._rotation_step = rotation_step
        self._grip_step = grip_step

        self._get_pose = self._lookup('get_pose')()

    def operate(self):

        # get current pose of EE
        status, pose_msg = self._get_pose.operate()
        current_pose = pose_msg_to_dct(pose_msg)

        # get current grip state
        current_grip = 0 #TODO
        prev_grip = current_grip #TODO

        # start reading current until requested to quit
        running = status
        pose_op = False
        grip_op = False
        while running:

            # Read character
            key = readchar.readkey()
            if key == 'w':
                pose_op = True
                current_pose['position']['y'] += self._position_step
            elif key == 's':
                pose_op = True
                current_pose['position']['y'] -= self._position_step
            elif key == 'a':
                pose_op = True
                current_pose['position']['x'] -= self._position_step
            elif key == 'd':
                pose_op = True
                current_pose['position']['x'] += self._position_step
            elif key == 'q':
                pose_op = True
                current_pose['position']['z'] += self._position_step
            elif key == 'z':
                pose_op = True
                current_pose['position']['z'] -= self._position_step
            elif key == 'i':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[1] = rot[1] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'k':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[1] = rot[1] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'j':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[0] = rot[0] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'l':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[0] = rot[0] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'u':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[2] = rot[2] + self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'm':
                pose_op = True
                rot = tf.transformations.euler_from_quaternion(self._pack_quaternion(current_pose['orientation']))
                rot[2] = rot[2] - self._rotation_step
                current_pose['orientation'] = tf.transformations.quaternion_from_euler(rot)
            elif key == 'g':
                grip_op = True
                current_grip += self._grip_step
            elif key == 'h':
                grip_op = True
                current_grip -= self._grip_step
            elif key == 'c':
                running = False
            else:
                continue

            # Move to Pose
            if pose_op:
                pose_op = False
                move = self._lookup('move')(current_pose['position'],current_pose['orientation'])
                status = move.operate()
                if not status:
                    running = False

            # Move gripper
            if grip_op:
                grip_op = False
                grip = self._lookup('grasp')(current_grip)
                _s = grip.operate()
                if not _s:
                    current_grip = prev_grip
                else:
                    prev_grip = current_grip

        return status

    def _pack_quaternion(self,dct):
        return [dct['x'],dct['y'],dct['z'],dct['w']]

class GeneralMovementBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, parent=None):
        super(GeneralMovementBehaviorPrimitives,self).__init__(parent)

    def instantiate_from_dict(self, dct, **kwargs):
        name = dct['name']
        if name == PrimitiveEnum.JOG.value:
            return Jog(self.lookup,dct['position_step'],dct['rotation_step'],dct['grip_step'])
        elif self.parent != None:
            return self.parent.instantiate_from_dict(dct,**kwargs)
        else:
            raise Exception('Invalid behavior primitive supplied')

    def lookup(self, primitive_type):
        if primitive_type == PrimitiveEnum.JOG.value:
            return Jog
        elif self.parent != None:
            return self.parent.lookup(primitive_type)
        else:
            raise Exception('Invalid behavior primitive supplied')
