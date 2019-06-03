'''
Primitives Relaxed-IK
Author: Curt Henrichs
Date: 5-2-19

Task primitives for runner implemented in Relaxed-IK.

While MoveIt provides many nice features to get started in developing primitives,
it is lacking in several aspects. Namely, repeatability of a task plan with same
joint paths. Relaxed-IK attempts to solve this problem by treating the pose goal
as an optimization problem where trade-offs in accuracy and joint travel are
weighed.

Using Wisconsin HCI's robot behavior package (developed for Authr), relaxed-ik was
successfuly integrated for all moveit supported primitives for the runner.

Primitve List:
    - grasp
            "Move the gripper to state specfied as target"
    - release
            "move the gripper, (identical to grasp)"
    - move
            "Commands robot to move end-effector to provided pose goal"
    - wait
            "Provides either interaction button or time based delay"
    - logger
            "Provides print functionality while debugging plan"
    - connect-object-to-robot
            "Currently unsupported, though intention is collision interfacing"
    - disconnect-object-from-robot
            "Currently unsupported, though intention is collision interfacing"
'''
import tf
import time
import rospy

from enum import Enum
from std_msgs.msg import Header
from tools.pose_conversion import *
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Pose, Point, Quaternion
from tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive

from behavior_execution.planners.relaxedik import RelaxedIKPlanner


POSE_OFFSET = Pose(position=Point(),orientation=Quaternion())


planner = RelaxedIKPlanner(
    {"follow_joint_trajectory":"gripper_command"},
    {"follow_joint_trajectory":[
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint']},
    pose_offsets={"follow_joint_trajectory": POSE_OFFSET})


class PrimitiveEnum(Enum):
    GRASP = 'grasp'
    RELEASE = 'release'
    MOVE = 'move'
    CONNECT_OBJECT_TO_ROBOT = 'connect_object'
    DISCONNECT_OBJECT_FROM_ROBOT = 'disconnect_object'
    PICK_AND_PLACE_VISION = 'pick_and_place_vision'
    PICK_AND_PLACE_STATIC = 'pick_and_place_static'
    CALIBRATE_ROBOT_TO_CAMERA = 'calibrate_robot_to_camera'


class ConnectObjectToRobot(Primitive):

    def __init__(self,object_name):
        self.box_name = object_name

    def operate(self):
        # TODO
        return True

class DisconnectObjectFromRobot(Primitive):

    def __init__(self,object_name):
        self.box_name = object_name

    def operate(self):
        # TODO
        return True

class Grasp(Primitive):

    def __init__(self, effort=1):
        self._plan = planner.plan_gripper_state('follow_joint_trajectory',[effort])

    def operate(self):
        return planner.execute('follow_joint_trajectory',self._plan)

class Release(Primitive):

    def __init__(self, effort=1):
        self._plan = planner.plan_gripper_state('follow_joint_trajectory',[effort])

    def operate(self):
        return planner.execute('follow_joint_trajectory',self._plan)

class Move(Primitive):

    def __init__(self, position, orientation):
        # Convert from dictionary to Pose
        self._pose = pose_dct_to_msg({'position':position,'orientation':orientation})

    def operate(self):
        plan = planner.plan_ee_pose('follow_joint_trajectory',self._pose)
        return planner.execute('follow_joint_trajectory',plan)

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
            ee_pose = planner.get_ee_pose('follow_joint_trajectory')
            status = self._envClient.calibrate_robot_to_camera('',ee_pose,self._transform)

        return status


class RelaxedIKBehaviorPrimitives(AbstractBehaviorPrimitives):

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
