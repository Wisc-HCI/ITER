
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

#TODO figure out gripper for grasp primitives

import tf
import sys
import time
import rospy
import moveit_commander

from enum import Enum
from std_msgs.msg import Header
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


ARM_MOVE_GROUP = rospy.get_param("arm_move_group")
GRIPPER_MOVE_GROUP = rospy.get_param("gripper_move_group")

moveit_commander.roscpp_initialize(sys.argv)
move_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


# Generate scene geometry
time.sleep(10)

scene.remove_world_object("table")
scene.add_box(
    name="table",
    pose=PoseStamped(
        header=Header(frame_id=robot.get_planning_frame()),
        pose=Pose(position=Point(x=0, y=0.44 ,z=-0.025), orientation=Quaternion(x=0, y=0, z=0, w=1))),
    size=(2, 1, 0.01))

rospy.sleep(1)
rospy.loginfo(scene.get_known_object_names())


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
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
        global GRIPPER_MOVE_GROUP
        #self.move_group_commander = moveit_commander.MoveGroupCommander(GRIPPER_MOVE_GROUP)
        pass

    def operate(self):
        return True


class Release(Primitive):

    def __init__(self):
        global GRIPPER_MOVE_GROUP
        #self.move_group_commander = moveit_commander.MoveGroupCommander(GRIPPER_MOVE_GROUP)
        pass

    def operate(self):
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

        print 'Orientation: ', self._pose.orientation

    def operate(self):
        global move_group_commander

        move_group_commander.clear_pose_targets()
        move_group_commander.set_pose_target(self._pose)
        retVal = move_group_commander.go(wait=True)
        move_group_commander.stop()
        return retVal


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
        return Release()
    elif name == PrimitiveEnum.MOVE.value:
        return Move(obj['position'],obj['orientation'])
    elif name == PrimitiveEnum.WAIT.value:
        return Wait(button_cb=button_callback, **obj)
    else:
        raise Exception('Invalid behavior primitive supplied')
