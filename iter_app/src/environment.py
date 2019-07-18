#!/usr/bin/env python

'''
Environment Node
Author Curt Henrichs
Date 5-16-19

Provides environment context for ITER runner.

'''


# __MODES__ for environment object type
MODE_COLLISION_MOVEIT = 'collision_moveit'
MODE_MARKER = 'marker'


import tf
import json
import time
import uuid
import rospy

from tf.transformations import *
from visualization_msgs.msg import *
from iter_vision.msg import CalibrationTf
from iter_app.msg import EnvironmentObject
from std_msgs.msg import Header, ColorRGBA
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose, Vector3, Quaternion, TransformStamped, PoseStamped

from iter_app.srv import GetARTagPose, GetARTagPoseResponse
from iter_app.srv import SetVisionParams, SetVisionParamsResponse
from iter_app.srv import GetVisionObject, GetVisionObjectResponse
from iter_app.srv import ClearTaskObjects, ClearTaskObjectsResponse
from iter_app.srv import ConnectTaskObject, ConnectTaskObjectResponse
from iter_app.srv import ReleaseTaskObject, ReleaseTaskObjectResponse
from iter_app.srv import GenerateTaskObjects, GenerateTaskObjectsResponse
from iter_app.srv import GetEnvironmentState, GetEnvironmentStateResponse
from iter_app.srv import CalibrateRobotToCamera, CalibrateRobotToCameraResponse

from iter_vision.srv import GetTagPose, GetTagPoseRequest
from iter_vision.srv import SetCalibrationTfs, SetCalibrationTfsRequest

rospy.init_node('environment')

mode = rospy.get_param('~mode',MODE_MARKER)
if mode == MODE_COLLISION_MOVEIT:
    import iter_app_tools.environment_interface.collision_moveit as task_env
elif mode == MODE_MARKER:
    import iter_app_tools.environment_interface.marker as task_env
else:
    raise Exception('Invalid environment mode selected')

import iter_app_tools.environment_interface.vision as vision_env


class Environment:

    def __init__(self,calibrate_ar_tag_id):

        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._calibrate_ar_tag_id = calibrate_ar_tag_id
        self._get_cam_pose = rospy.ServiceProxy('/robot_camera_align/get_tag_pose',GetTagPose)
        self._set_tfs = rospy.ServiceProxy('/robot_camera_align/set_tfs',SetCalibrationTfs)

        self._set_vision_params_srv = rospy.Service("/environment/set_vision_params",SetVisionParams,self._set_vision_params)
        self._gen_task_objs_srv = rospy.Service("/environment/generate_task_objects",GenerateTaskObjects,self._generate_task_objs)
        self._clear_task_objs_srv = rospy.Service("/environment/clear_task_objects",ClearTaskObjects,self._clear_task_objs)
        self._connect_task_obj_srv = rospy.Service("/environment/connect_task_object",ConnectTaskObject,self._connect_task_obj)
        self._release_task_obj_srv = rospy.Service("/environment/release_task_object",ReleaseTaskObject,self._release_task_obj)
        self._get_vision_obj_srv = rospy.Service("/environment/get_vision_object",GetVisionObject,self._get_vision_obj)
        self._cal_bot_to_cam_srv = rospy.Service("/environment/calibrate_robot_to_camera",CalibrateRobotToCamera,self._cal_bot_to_cam)
        self._get_state_srv = rospy.Service("/environment/get_state",GetEnvironmentState,self._get_state)
        self._get_ar_tag_pose = rospy.Service("/environment/get_ar_tag_pose",GetARTagPose,self._get_ar_tag_pose)

        # manual calibration marker
        self._interactive_marker_server = InteractiveMarkerServer("interactive_markers")
        self._calibration_marker = self._create_manual_calibration_marker((0,0,0),(0,0,0,1))
        self._interactive_marker_server.insert(self._calibration_marker,self._process_cam_marker_feedback)
        self._interactive_marker_server.applyChanges()

    def _create_manual_calibration_marker(self,pos,rot):
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "calibration_point_1"
        interactive_marker.name = "camera_marker"
        interactive_marker.description = "Camera TF rotation marker"
        interactive_marker.scale = 0.1
        interactive_marker.pose = Pose(position=Vector3(x=pos[0],y=pos[1],z=pos[2]),
                                       orientation=Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]))
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale = Vector3(x=0.05,y=0.05,z=0.05)
        box_marker.color = ColorRGBA(r=0.5,g=05,b=0.5,a=0.75)

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        interactive_marker.controls.append(box_control)

        controls = {
            'rotate_x': {'orientation': Quaternion(x=1,y=0,z=0,w=1), 'mode': InteractiveMarkerControl.ROTATE_AXIS},
            'rotate_y': {'orientation': Quaternion(x=0,y=0,z=1,w=1), 'mode': InteractiveMarkerControl.ROTATE_AXIS},
            'rotate_z': {'orientation': Quaternion(x=0,y=1,z=0,w=1), 'mode': InteractiveMarkerControl.ROTATE_AXIS},
            'move_x': {'orientation': Quaternion(x=1,y=0,z=0,w=1), 'mode': InteractiveMarkerControl.MOVE_AXIS},
            'move_y': {'orientation': Quaternion(x=0,y=0,z=1,w=1), 'mode': InteractiveMarkerControl.MOVE_AXIS},
            'move_z': {'orientation': Quaternion(x=0,y=1,z=0,w=1), 'mode': InteractiveMarkerControl.MOVE_AXIS}
        }
        for key in controls.keys():
            control = InteractiveMarkerControl()
            control.name = key
            control.orientation = controls[key]['orientation']
            control.interaction_mode = controls[key]['mode']
            interactive_marker.controls.append(control)

        return interactive_marker

    def _process_cam_marker_feedback(self, feedback):
        print feedback.pose
        pos, rot = self._pose_msg_to_tf(feedback.pose)
        self._calibration_tfs['c1_to_c2']['position'] = pos
        self._calibration_tfs['c1_to_c2']['rotation'] = rot

    def _pose_msg_to_tf(self,msg):
        pos = (msg.position.x,msg.position.y,msg.position.z)
        rot = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        return pos, rot

    def _generate_task_objs(self, request):
        # Generates new markers of objects defined by array of objects provided
        return GenerateTaskObjectsResponse(status=task_env.generate_dynamic_environment(request.objects))

    def _clear_task_objs(self, request):
        # Clears set of objects defined by array of string IDs
        return ClearTaskObjectsResponse(status=task_env.clear_dynamic_environment(request.ids,request.all))

    def _connect_task_obj(self, request):
        # Connects object to robot
        # Provide a pose which is used for release to calculate the transformation
        # over movement used to plot new object
        status = task_env.connect_obj_to_robot(request.id,request.pose)
        return ConnectTaskObjectResponse(status=status)

    def _release_task_obj(self, request):
        # Disconnects object from robot
        # Provide a pose which is used to calculate the transformation over
        # movement used to plot new object
        status = task_env.disconnect_obj_from_robot(request.id,request.pose)
        return ReleaseTaskObjectResponse(status=status)

    def _get_vision_obj(self, request):
        # Finds a object from vision set that meets the criteria given.
        # Converts to task object with ID.
        # Returns pose of object with ID.
        type = None
        if request.type == 'large':
            type = vision_env.BLOCK_LARGE
        elif request.type == 'small':
            type = vision_env.BLOCK_SMALL
        elif request.type == 'unknown':
            type = vision_env.BLOCK_UNKNOWN

        id, pose = vision_env.get_block(type)
        print '\n\n', pose, '\n\n'

        response = GetVisionObjectResponse()
        response.status = not id == None
        if not response.status:
            return response
        response.vision_id = 'block_{0}'.format(id)

        response.pose = self._tf_listener.transformPose(request.frame_id,PoseStamped(pose=pose,header=Header(frame_id='/map'))).pose

        response.task_id = response.vision_id + '_' + str(uuid.uuid1().hex)

        response.status = task_env.generate_dynamic_environment([EnvironmentObject(
            representation=EnvironmentObject.REPRESENTATION_BOX,
            id=response.task_id,
            size=Vector3(0.1,0.1,0.1), #Note, this is for representation only
            pose=response.pose
        )])

        return response

    def _cal_bot_to_cam(self, request):
        # probe camera to robot transform, note robot's ar tag must be within
        # camera's field of view

        # pre-process poses into tfs
        eePos, eeRot = self._pose_msg_to_tf(request.ee_pose)
        gtaPos, gtaRot = self._pose_msg_to_tf(request.tag_grip_tf)
        print '\n\n\n', gtaPos, '\n\n', gtaRot, '\n\n\n'

        # find calibration tag
        tagId = request.ar_tag_id if request.ar_tag_id != "" else self._calibrate_ar_tag_id

        status = True
        try:
            resp = self._get_cam_pose(tag_frame_id='ar_marker_{}'.format(tagId),duration=GetTagPoseRequest.DEFAULT_DURATION)
            status = resp.status
            tagPos = resp.position
            tagRot = resp.rotation
        except Exception, e:
            return CalibrateRobotToCameraResponse(status=False)

        # update calibration points
        if status:
            tfs = []
            tfs.append(CalibrationTfs(child_frame='calibration_point_1',parent_frame='base_link',position=eePos,rotation=eeRot))
            tfs.append(CalibrationTfs(child_frame='calibration_point_2',parent_frame='calibration_point_1',position=gtaPos,rotation=gtaRot))
            tfs.append(CalibrationTfs(child_frame='map',parent_frame='calibration_point_2',position=tagPos,rotation=tagRot))
            status = self._set_tfs(tfs).status

        # update interactive marker
        if status:
            self._calibration_marker.pose=Pose(
                position=Vector3(x=gtaPos[0],y=gtaPos[1],z=gtaPos[2]),
                orientation=Quaternion(x=gtaRot[0],y=gtaRot[1],z=gtaRot[2],w=gtaRot[3]))
            self._interactive_marker_server.applyChanges()

        return CalibrateRobotToCameraResponse(status=status)

    def _get_state(self, request):
        return GetEnvironmentState(
            grasped_task_objects=task_env.get_grasped_ids(),
            all_task_objects=task_env.get_all_task_ids(),
            all_vision_objects=vision_env.get_vision_ids(),
            all_ar_tags=vision_env.get_ar_ids())

    def _set_vision_params(self, request):
        params = json.loads(request.params)
        status = vision_env.set_vision_params(params)
        return SetVisionParamsResponse(status=status)

    def _get_ar_tag_pose(self, request):
        p_raw = vision_env.get_ar_tag(request.tag_id)
        status = p_raw != None

        pose = Pose()
        if status:
            p_tf = self._tf_listener.transformPose(request.frame_id,PoseStamped(pose=p_raw,header=Header(frame_id='/map'))).pose
            pose.position.x = p_tf.position.x + request.offset.position.x
            pose.position.y = p_tf.position.y + request.offset.position.y
            pose.position.z = p_tf.position.z + request.offset.position.z
            pose.orientation = request.offset.orientation

        response = GetARTagPoseResponse()
        response.status = status
        response.pose = pose
        return response


if __name__ == "__main__":
    calibrate_tag = rospy.get_param('~calibrate_ar_tag_id',None)
    env = Environment(calibrate_tag)
    rospy.spin()
