'''
Primitives Moveit
Author: Curt Henrichs
Date: 5-2-19

Task primitives for runner implemented in MoveIt.

MoveIt provides a robust and comprehensive interface to control a robotic arm
and thus is a natural choice to develop the first iteration of ITER's primitive
library.

Primitve List:
    - grasp
            "Move the gripper to state specfied as target"
    - release
            "move the gripper, (identical to grasp)"
    - move
            "Commands robot to move end-effector to provided pose goal"
    - connect-object-to-robot
            "Currently unsupported, though intention is collision interfacing"
    - disconnect-object-from-robot
            "Currently unsupported, though intention is collision interfacing"
    - pick-and-place-static
    - pick-and-place-vision
    - calibrate-robot-to-camera
'''

import tf
import sys
import time
import rospy
import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

from enum import Enum
from std_msgs.msg import Header
from tools.pose_conversion import *
from abc import ABCMeta, abstractmethod
from tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive


ARM_MOVE_GROUP = rospy.get_param("arm_move_group")
GRIPPER_MOVE_GROUP = rospy.get_param("gripper_move_group")
GRIPPER_TYPE = rospy.get_param("gripper_type")


arm_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
gripper_group_commander = moveit_commander.MoveGroupCommander(GRIPPER_MOVE_GROUP)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    CONNECT_OBJECT_TO_ROBOT = 'connect_object'
    DISCONNECT_OBJECT_FROM_ROBOT = 'disconnect_object'
    PICK_AND_PLACE_VISION = 'pick_and_place_vision'
    PICK_AND_PLACE_STATIC = 'pick_and_place_static'
    CALIBRATE_ROBOT_TO_CAMERA = 'calibrate_robot_to_camera'


class Grasp(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):

        arm_group_commander.clear_pose_targets()

        if GRIPPER_TYPE == 'mico-3':
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_2':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_3':self._effort})
        elif GRIPPER_TYPE == 'mico-2':
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_2':self._effort})
        else: # Robotiq 85
            joint = gripper_group_commander.get_joints()[0]
            gripper_group_commander.set_joint_value_target({joint:self._effort})

        gripper_group_commander.go(wait=True)
        gripper_group_commander.stop()
        return True

class Release(Primitive):

    def __init__(self, effort=1):
        self._effort = effort

    def operate(self):

        arm_group_commander.clear_pose_targets()

        if GRIPPER_TYPE == 'mico-3':
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_2':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s300_joint_finger_3':self._effort})
        elif GRIPPER_TYPE == 'mico-2':
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_1':self._effort})
            gripper_group_commander.set_joint_value_target({'m1n6s200_joint_finger_2':self._effort})
        else: # Robotiq 85
            joint = gripper_group_commander.get_joints()[0]
            gripper_group_commander.set_joint_value_target({joint:self._effort})

        gripper_group_commander.go(wait=True)
        gripper_group_commander.stop()
        return True

class Move(Primitive):

    def __init__(self, position, orientation):
        # Convert from dictionary to Pose
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        global arm_group_commander

        arm_group_commander.clear_pose_targets()
        arm_group_commander.set_pose_target(self._pose)
        retVal = arm_group_commander.go(wait=True)
        arm_group_commander.stop()
        return retVal

class ConnectObjectToRobot(Primitive):

    def __init__(self,object_name,envClient):
        self.box_name = object_name
        self._envClient = envClient

    def operate(self):
        #TODO
        #env.connect_obj_to_robot(self.box_name)
        return True

class DisconnectObjectFromRobot(Primitive):

    def __init__(self,object_name,envClient):
        self.box_name = object_name
        self._envClient = envClient

    def operate(self):
        #TODO
        #env.disconnect_obj_from_robot(self.box_name)
        return True

class PickAndPlaceVision(Primitive):

    def __init__(self,object_type,path_to_region,path_to_destination,grasp_effort,release_effort,envClient, **kwargs):
        self._type = object_type
        self._path_to_region = [Move(dct['position'],dct['orientation']) for dct in path_to_region]
        self._path_to_dest = [Move(dct['position'],dct['orientation']) for dct in path_to_destination]
        self._grasp = Grasp(grasp_effort)
        self._release = Release(release_effort)
        self._envClient = envClient

    def operate(self):
        status = True

        # find object, getid
        resp = self._envClient.get_vision_object(self._type,'base_link')
        status = resp.status

        # move to region
        if status:
            for m in self._path_to_region:
                status = m.operate()
                if not status:
                    break

        # get pose of object relative to base_link
        if status:
            dPose = pose_msg_to_dct(resp.pose)
            Move(dPose['position'],dPose['orientation']).operate()

        # grasp
        if status:
            status = ConnectObjectToRobot(resp.task_id,self._envClient).operate()
            status = status or self._grasp.operate()

        # move to destination
        if status:
            for m in self._path_to_dest:
                status = m.operate()
                if not status:
                    break

        # release
        if status:
            status = self._release.operate()
            status = status or DisconnectObjectFromRobot(resp.task_id,self._envClient).operate()

        return status

class PickAndPlaceStatic(Primitive):

    def __init__(self,path_to_object,path_to_destination,object_name,grasp_effort,release_effort,envClient, **kwargs):
        self._ops = []
        self._ops = self._ops + [Move(dct['position'],dct['orientation']) for dct in path_to_object]
        self._ops.append(ConnectObjectToRobot(object_name,envClient))
        self._ops.append(Grasp(grasp_effort))
        self._ops = self._ops + [Move(dct['position'],dct['orientation']) for dct in path_to_destination]
        self._ops.append(DisconnectObjectFromRobot(object_name,envClient))
        self._ops.append(Release(release_effort))

    def operate(self):
        status = True
        for op in self._ops:
            status = op.operate()
            if not status:
                break
        return status

class CalibrateRobotToCamera(Primitive):

    def __init__(self,envClient,path_to_region,tag_to_ee_transform):
        self._envClient = envClient
        self._path = [Move(dct['position'],dct['orientation']) for dct in path_to_region]
        self._transform = pose_dct_to_msg(tag_to_ee_transform)

    def operate(self):
        status = True

        # place robot into camera field of view
        for m in path:
            status = m.operate()
            if not status
                break

        # run calibration routine
        if status:
            ee_pose = arm_group_commander.get_current_pose().pose
            status = self._envClient.calibrate_robot_to_camera('',ee_pose,self._transform)

        return status

class MoveItBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, envClient):
        super(AbstractBehaviorPrimitives,self).__init__(envClient)

    def instantiate_from_dict(obj, button_callback):

        name = obj['name']
        if name == PrimitiveEnum.GRASP.value:
            return Grasp(obj['effort'])
        elif name == PrimitiveEnum.RELEASE.value:
            return Release(obj['effort'])
        elif name == PrimitiveEnum.MOVE.value:
            return Move(obj['position'],obj['orientation'])
        elif name == PrimitiveEnum.CONNECT_OBJECT_TO_ROBOT.value:
            return ConnectObjectToRobot(obj['object_name'],self._envClient)
        elif name == PrimitiveEnum.DISCONNECT_OBJECT_FROM_ROBOT.value:
            return DisconnectObjectFromRobot(obj['object_name'],self._envClient)
        elif name == PrimitiveEnum.PICK_AND_PLACE_VISION.value:
            return PickAndPlaceVision(envClient=self._envClient, **obj)
        elif name == PrimitiveEnum.PICK_AND_PLACE_STATIC.value:
            return PickAndPlaceStatic(envClient=self._envClient, **obj)
        elif name == PrimitiveEnum.CALIBRATE_ROBOT_TO_CAMERA.value:
            return CalibrateRobotToCamera(self._envClient,obj['path_to_region'],obj['tag_to_ee_transform'])
        else:
            return super(AbstractBehaviorPrimitives,self).instantiate_from_dict(obj,button_callback)
