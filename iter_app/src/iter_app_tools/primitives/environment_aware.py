'''
Environment Aware Primitive
    - connect-object-to-robot
            "Currently unsupported, though intention is collision interfacing"
    - disconnect-object-from-robot
            "Currently unsupported, though intention is collision interfacing"
'''

import tf
import json
import yaml
import rospy

from enum import Enum
from iter_app_tools.pose_conversion import *
from iter_app_tools.primitives.abstract import AbstractBehaviorPrimitives, Primitive, ReturnablePrimitive


class PrimitiveEnum(Enum):
    FIND_VISION_OBJECT = 'find_vision_object'
    CONNECT_OBJECT_TO_ROBOT = 'connect_object'
    DISCONNECT_OBJECT_FROM_ROBOT = 'disconnect_object'
    PICK_AND_PLACE_VISION = 'pick_and_place_vision'
    PICK_AND_PLACE_STATIC = 'pick_and_place_static'
    CALIBRATE_ROBOT_TO_CAMERA = 'calibrate_robot_to_camera'
    MOVE_AR_TAG_POSE = 'move_ar_tag_pose'
    CALIBRATE_VISION_POSE = 'calibrate_vision_pose'


class FindVisionObject(ReturnablePrimitive):

    def __init__(self,object_type,vision_params,envClient,disable_offset=False):
        self._type = object_type
        self._envClient = envClient
        self._visionParams = vision_params
        self._disable_offset = disable_offset

    def operate(self):
        # set vision parameters & wait
        status = self._envClient.set_vision_params(json.dumps(self._visionParams))
        rospy.sleep(3)

        # find object
        if status:
            resp = self._envClient.get_vision_object(self._type,'base_link',self._disable_offset)
            status = resp.status
            data = (resp.task_id, resp.pose)
        else:
            data = ('',None)

        # attempt to set vision parameters to default
        self._envClient.set_vision_params(json.dumps({'default':True}))

        return status, data

class ConnectObjectToRobot(Primitive):

    def __init__(self,object_name,envClient,lookup):
        self._id = object_name
        self._envClient = envClient
        self._get_pose = lookup('get_pose')()

    def operate(self):
        status, ee_pose = self._get_pose.operate()
        status = status and self._envClient.connect_task_object(self._id,ee_pose).status
        print 'status', status
        return status

class DisconnectObjectFromRobot(Primitive):

    def __init__(self,object_name,envClient,lookup):
        self._id = object_name
        self._envClient = envClient
        self._get_pose = lookup('get_pose')()

    def operate(self):
        status, ee_pose = self._get_pose.operate()
        status = status and self._envClient.release_task_object(self._id,ee_pose).status
        print 'status', status
        return status

class PickAndPlaceVision(Primitive):

    def __init__(self,object_type,path_to_region,path_to_destination,grasp_effort,release_effort,grasp_offset,safe_height,vision_params,envClient,lookup, **kwargs):
        self._find_obj = lookup('find_vision_object')(object_type,vision_params,envClient)
        self._path_to_region = [lookup('move')(dct['position'],dct['orientation']) for dct in path_to_region]
        self._path_to_dest = [lookup('move')(dct['position'],dct['orientation']) for dct in path_to_destination]
        self._grasp = lookup('grasp')(grasp_effort)
        self._release = lookup('release')(release_effort)
        self._envClient = envClient
        self._lookup = lookup
        self._safe_height = safe_height
        self._grasp_offset = pose_dct_to_msg(grasp_offset)

    def operate(self):
        status = True

        # find object, getid
        status, (obj_id, pose) = self._find_obj.operate()

        # apply offset
        print self._grasp_offset
        position = self._grasp_offset.position
        orientation = self._grasp_offset.orientation
        offset_pos = [position.x,position.y,position.z]
        offset_rot = [orientation.x,orientation.y,orientation.z,orientation.w]

        print pose
        position = pose.position
        orientation = pose.orientation
        object_pos = [position.x,position.y,position.z]
        object_rot = [orientation.x,orientation.y,orientation.z,orientation.w]

        grasp_pos = [object_pos[i] + offset_pos[i] for i in range(0,len(object_pos))]
        #grasp_pos[2] = offset_pos[2] # fixed-z-offset? TODO actual height measure later

        #grasp_rot = tf.transformations.quaternion_multiply(object_rot,offset_rot)
        grasp_rot = offset_rot #TODO handle actual angle later. Just need to see if the pose position is even close

        grasp_pose = Pose(position=Vector3(x=grasp_pos[0],y=grasp_pos[1],z=grasp_pos[2]),
                          orientation=Quaternion(x=grasp_rot[0],y=grasp_rot[1],z=grasp_rot[2],w=grasp_rot[3]))
        print grasp_pose

        # move to region
        if status:
            for m in self._path_to_region:
                status = m.operate()
                if not status:
                    break

        # get pose of object relative to base_link
        dPose = pose_msg_to_dct(grasp_pose)
        if status:

            # safe-height movement
            dPose['position']['z'] += self._safe_height
            self._lookup('move')(dPose['position'],dPose['orientation']).operate()

            # object location movement
            dPose['position']['z'] -= self._safe_height
            self._lookup('move')(dPose['position'],dPose['orientation']).operate()

        # grasp
        if status:
            status = ConnectObjectToRobot(obj_id,self._envClient,self._lookup).operate()
            status = status and self._grasp.operate()

        # safe-height movement after grasp
        if status:
            dPose['position']['z'] += self._safe_height
            self._lookup('move')(dPose['position'],dPose['orientation']).operate()

        # move to destination
        if status:
            for m in self._path_to_dest:
                status = m.operate()
                if not status:
                    break

        # release
        if status:
            status = self._release.operate()
            status = status and DisconnectObjectFromRobot(obj_id,self._envClient,self._lookup).operate()

        return status

class PickAndPlaceStatic(Primitive):

    def __init__(self,path_to_object,path_to_destination,object_name,grasp_effort,release_effort,envClient,lookup, **kwargs):
        self._ops = []
        self._ops = self._ops + [lookup('move')(**dct) for dct in path_to_object]
        self._ops.append(ConnectObjectToRobot(object_name,envClient,lookup))
        self._ops.append(lookup('grasp')(grasp_effort))
        self._ops = self._ops + [lookup('move')(**dct) for dct in path_to_destination]
        self._ops.append(DisconnectObjectFromRobot(object_name,envClient,lookup))
        self._ops.append(lookup('release')(release_effort))

    def operate(self):
        status = True
        for op in self._ops:
            status = op.operate()
            if not status:
                break
        return status

class CalibrateRobotToCamera(Primitive):

    def __init__(self,envClient,path_to_region,tag_to_ee_transform,lookup,tag_id='', **kwargs):
        self._envClient = envClient
        self._tag_id = tag_id
        self._path = [lookup('move')(dct['position'],dct['orientation']) for dct in path_to_region]
        self._transform = pose_dct_to_msg(tag_to_ee_transform)
        self._get_pose = lookup('get_pose')()

    def operate(self):
        status = True

        # place robot into camera field of view
        print 'positioning...'
        for m in self._path:
            status = m.operate()
            if not status:
                break

        # give some time for vision system to process
        print 'waiting...'
        rospy.sleep(15)

        # run calibration routine
        print 'calibrating...'
        if status:
            status, ee_pose = self._get_pose.operate()
            status = status and self._envClient.calibrate_robot_to_camera(self._tag_id,ee_pose,self._transform).status

        print status
        return status

class MoveARTagPose(Primitive):

    def __init__(self, envClient, lookup, tag_id, offset, **kwargs):
        self._envClient = envClient
        self._lookup = lookup
        self._tag_id = tag_id
        self._offset = pose_dct_to_msg(offset)

    def operate(self):

        resp = self._envClient.get_ar_tag_pose(self._tag_id, 'base_link', self._offset)
        status = resp.status

        if status:
            pdct = pose_msg_to_dct(resp.pose)
            move = self._lookup('move')(pdct['position'],pdct['orientation'])
            status = move.operate()

        return status

class CalibrateVisionPose(Primitive):

    def __init__(self, envClient, lookup, step, object_type, vision_params, grasp_offset, safe_height, home_pose, **kwargs):
        self._step = step
        self._safe_height = safe_height
        self._lookup = lookup

        self._block_prompt = lookup('prompt')('Ready for block trial?')
        self._loop_prompt = lookup('prompt')('Run again?')
        self._get_obj = lookup('find_vision_object')(object_type,vision_params,envClient,True)
        self._jog = lookup('jog')(lookup, self._step, self._step, 0.02)
        self._get_pose = lookup('get_pose')()
        self._grasp_offset = pose_dct_to_msg(grasp_offset)
        self._move_home = lookup('move')(home_pose['position'],home_pose['orientation'])

    def operate(self):
        pre_poses = []
        post_poses = []

        # collect calibration points
        running = True
        while running:

            # prompt user to place block
            status, response = self._block_prompt.operate()
            if not status:
                running = False
            print 'starting'

            # attempt to find vision object
            _s, (obj_id, v_pose) = self._get_obj.operate()
            if not _s:
                print 'Could not find vision object'
            else:
                print 'calculating pose'
                # apply offset
                print self._grasp_offset
                position = self._grasp_offset.position
                orientation = self._grasp_offset.orientation
                offset_pos = [position.x,position.y,position.z]
                offset_rot = [orientation.x,orientation.y,orientation.z,orientation.w]

                print v_pose
                position = v_pose.position
                orientation = v_pose.orientation
                object_pos = [position.x,position.y,position.z]
                object_rot = [orientation.x,orientation.y,orientation.z,orientation.w]

                grasp_pos = [object_pos[i] + offset_pos[i] for i in range(0,len(object_pos))]
                grasp_pos[2] = offset_pos[2] # fixed-z-offset? TODO actual height measure later

                #grasp_rot = tf.transformations.quaternion_multiply(object_rot,offset_rot)
                grasp_rot = offset_rot #TODO handle actual angle later. Just need to see if the pose position is even close

                grasp_pose = Pose(position=Vector3(x=grasp_pos[0],y=grasp_pos[1],z=grasp_pos[2]),
                                  orientation=Quaternion(x=grasp_rot[0],y=grasp_rot[1],z=grasp_rot[2],w=grasp_rot[3]))
                print grasp_pose

                # move toward vision object
                print 'moving toward object'
                dPose = pose_msg_to_dct(grasp_pose)
                dPose['position']['z'] += self._safe_height
                self._lookup('move')(dPose['position'],dPose['orientation']).operate()
                dPose['position']['z'] -= self._safe_height
                self._lookup('move')(dPose['position'],dPose['orientation']).operate()

                # allow user jog pose
                print "Jog robot"
                self._jog.operate()
                _s, j_pose = self._get_pose.operate() # note, removing grasp offset to find block location
                j_pose.position.x -= self._grasp_offset.position.x
                j_pose.position.y -= self._grasp_offset.position.y
                j_pose.position.z -= self._grasp_offset.position.z

                # save pose pair
                print 'Saving pose info'
                pre_poses.append(v_pose)
                post_poses.append(j_pose)

            # prompt run again?
            status, response = self._loop_prompt.operate()
            response.lower()
            if not status:
                running = False
            elif response == 'n' or response == 'no':
                running = False
            print '\n\nOK!\n'

            # regardless, return to home position
            self._move_home.operate()

        # provide pose pairs for calibration in YAML form
        data = {'initial': [],'offset': []}
        for pose in pre_poses:
            pos = pose.position
            rot = pose.orientation
            data['initial'].append({'position':{'x':pos.x,'y':pos.y,'z':pos.z},
                                    'orientation':{'x':rot.x,'y':rot.y,'z':rot.z,'w':rot.w}})
        for pose in post_poses:
            pos = pose.position
            rot = pose.orientation
            print pos.x
            data['offset'].append({'position':{'x':pos.x.item(),'y':pos.y.item(),'z':pos.z.item()},
                                   'orientation':{'x':rot.x.item(),'y':rot.y.item(),'z':rot.z.item(),'w':rot.w.item()}})
        _y = yaml.dump(data, default_flow_style=False)

        print '\n\n', _y, '\n\n'
        return status


class EnvironmentAwareBehaviorPrimitives(AbstractBehaviorPrimitives):

    def __init__(self, envClient, parent=None):
        super(EnvironmentAwareBehaviorPrimitives,self).__init__(parent)
        self._envClient = envClient

    def instantiate_from_dict(self, dct, **kwargs):

        name = dct['name']
        if name == PrimitiveEnum.FIND_VISION_OBJECT.value:
            return FindVisionObject(envClient=self._envClient, **dct)
        elif name == PrimitiveEnum.CONNECT_OBJECT_TO_ROBOT.value:
            return ConnectObjectToRobot(dct['object_name'], self._envClient, self.lookup)
        elif name == PrimitiveEnum.DISCONNECT_OBJECT_FROM_ROBOT.value:
            return DisconnectObjectFromRobot(dct['object_name'], self._envClient, self.lookup)
        elif name == PrimitiveEnum.PICK_AND_PLACE_VISION.value:
            return PickAndPlaceVision(envClient=self._envClient, lookup=self.lookup, **dct)
        elif name == PrimitiveEnum.PICK_AND_PLACE_STATIC.value:
            return PickAndPlaceStatic(envClient=self._envClient, lookup=self.lookup, **dct)
        elif name == PrimitiveEnum.CALIBRATE_ROBOT_TO_CAMERA.value:
            return CalibrateRobotToCamera(envClient=self._envClient, lookup=self.lookup, **dct)
        elif name == PrimitiveEnum.MOVE_AR_TAG_POSE.value:
            return MoveARTagPose(envClient=self._envClient, lookup=self.lookup, **dct)
        elif name == PrimitiveEnum.CALIBRATE_VISION_POSE.value:
            return CalibrateVisionPose(envClient=self._envClient, lookup=self.lookup, **dct)
        elif self.parent != None:
            return self.parent.instantiate_from_dict(dct,**kwargs)
        else:
            raise Exception('Invalid behavior primitive supplied')

    def lookup(self, primitive_type):
        if primitive_type == PrimitiveEnum.FIND_VISION_OBJECT.value:
            return FindVisionObject
        elif primitive_type == PrimitiveEnum.CONNECT_OBJECT_TO_ROBOT.value:
            return ConnectObjectToRobot
        elif primitive_type == PrimitiveEnum.DISCONNECT_OBJECT_FROM_ROBOT.value:
            return DisconnectObjectFromRobot
        elif primitive_type == PrimitiveEnum.PICK_AND_PLACE_VISION.value:
            return PickAndPlaceVision
        elif primitive_type == PrimitiveEnum.PICK_AND_PLACE_STATIC.value:
            return PickAndPlaceStatic
        elif primitive_type == PrimitiveEnum.CALIBRATE_ROBOT_TO_CAMERA.value:
            return CalibrateRobotToCamera
        elif primitive_type == PrimitiveEnum.MOVE_AR_TAG_POSE.value:
            return MoveARTagPose
        elif primitive_type == PrimitiveEnum.CALIBRATE_VISION_POSE.value:
            return CalibrateVisionPose
        elif self.parent != None:
            return self.parent.lookup(primitive_type)
        else:
            raise Exception('Invalid behavior primitive supplied')
