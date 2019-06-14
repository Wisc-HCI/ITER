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
import time
import uuid
import rospy
import tf2_ros

from tf.transformations import *
from visualization_msgs.msg import *
from iter_app.msg import EnvironmentObject
from std_msgs.msg import Header, ColorRGBA
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose, Vector3, Quaternion, TransformStamped

from iter_app.srv import GetVisionObject, GetVisionObjectResponse
from iter_app.srv import ClearTaskObjects, ClearTaskObjectsResponse
from iter_app.srv import ConnectTaskObject, ConnectTaskObjectResponse
from iter_app.srv import ReleaseTaskObject, ReleaseTaskObjectResponse
from iter_app.srv import GenerateTaskObjects, GenerateTaskObjectsResponse
from iter_app.srv import GetEnvironmentState, GetEnvironmentStateResponse
from iter_app.srv import CalibrateRobotToCamera, CalibrateRobotToCameraResponse

from iter_vision.srv import GetTagPose, GetTagPoseRequest

rospy.init_node('environment')

mode = rospy.get_param('~mode',MODE_MARKER)
if mode == MODE_COLLISION_MOVEIT:
    import iter_app_tools.environment_interface.collision_moveit as task_env
elif mode == MODE_MARKER:
    import iter_app_tools.environment_interface.marker as task_env
else:
    raise Exception('Invalid environment mode selected')

import iter_app_tools.environment_interface.vision as vision_env


DEFAULT_TF = {
        'position': (-0.0498019680381, 0.872681796551, 0.34569761157),
        'orientation': (-0.0167846251279,-0.976989865303,0.2126237005,0.000687647901941)
    }

'''
position:
  x: -0.0498019680381
  y: 0.872681796551
  z: 0.34569761157
orientation:
  x: -0.0167846251279
  y: -0.976989865303
  z: 0.2126237005
  w: 0.000687647901941

'''


CALIBRATE_TAG_WAIT_TIMEOUT = 30


class Environment:

    def __init__(self,calibrate_ar_tag_id):
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._calibrate_ar_tag_id = calibrate_ar_tag_id
        self._get_cam_pose = rospy.ServiceProxy('/robot_camera_align/get_tag_pose',GetTagPose)
        self._update_baselink_map_tf(DEFAULT_TF['position'],DEFAULT_TF['orientation'])

        self._gen_task_objs_srv = rospy.Service("/environment/generate_task_objects",GenerateTaskObjects,self._generate_task_objs)
        self._clear_task_objs_srv = rospy.Service("/environment/clear_task_objects",ClearTaskObjects,self._clear_task_objs)
        self._connect_task_obj_srv = rospy.Service("/environment/connect_task_object",ConnectTaskObject,self._connect_task_obj)
        self._release_task_obj_srv = rospy.Service("/environment/release_task_object",ReleaseTaskObject,self._release_task_obj)
        self._get_vision_obj_srv = rospy.Service("/environment/get_vision_object",GetVisionObject,self._get_vision_obj)
        self._cal_bot_to_cam_srv = rospy.Service("/environment/calibrate_robot_to_camera",CalibrateRobotToCamera,self._cal_bot_to_cam)
        self._get_state_srv = rospy.Service("/environment/get_state",GetEnvironmentState,self._get_state)

        self._interactive_marker_server = InteractiveMarkerServer("interactive_markers")

        self._camera_marker = InteractiveMarker()
        self._camera_marker.header.frame_id = "base_link"
        self._camera_marker.name = "camera_marker"
        self._camera_marker.description = "Camera TF rotation marker"
        self._camera_marker.scale = 0.5
        self._camera_marker.pose = Pose(position=Vector3(x=DEFAULT_TF['position'][0],
                                                         y=DEFAULT_TF['position'][1],
                                                         z=DEFAULT_TF['position'][2]),
                                        orientation=Quaternion(x=DEFAULT_TF['orientation'][0],
                                                               y=DEFAULT_TF['orientation'][1],
                                                               z=DEFAULT_TF['orientation'][2],
                                                               w=DEFAULT_TF['orientation'][3]))

        print self._camera_marker

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale = Vector3(x=0.1,y=0.1,z=0.1)
        box_marker.color = ColorRGBA(r=0.5,g=05,b=0.5,a=0.75)

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        self._camera_marker.controls.append(box_control)

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
            self._camera_marker.controls.append(control)

        self._interactive_marker_server.insert(self._camera_marker,self._process_cam_marker_feedback)
        self._interactive_marker_server.applyChanges()

    def _process_cam_marker_feedback(self, feedback):
        print feedback.pose
        pos, rot = self._pose_msg_to_tf(feedback.pose)
        self._update_baselink_map_tf(pos,rot)

    def _update_baselink_map_tf(self,pos,rot):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'map'

        msg.transform.translation = Vector3(x=pos[0],y=pos[1],z=pos[2])
        msg.transform.rotation = Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3])

        self._tf_broadcaster.sendTransform(msg)

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
        id = vision_env.get_block(type)

        response = GetVisionObjectResponse()
        response.status = not id is None
        if not response.status:
            return response
        response.vision_id = 'block_{0}'.format(id)

        (pos, rot) = self._tf_listener.lookupTransform(response.vision_id,request.frame_id, rospy.Time(0))
        response.pose = Pose(position=Vector3(x=pos[0],y=pos[1],z=pos[2]),
                             orientation=Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]))

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

        #TODO this is in development and currently broken. Furture work to
        # finish  / fix this.
        # Essentially what needs to be done, is calculate correct position and orientation

        print 'in env calibration'

        # find calibration tag
        tagId = request.ar_tag_id if request.ar_tag_id != "" else str(self._calibrate_ar_tag_id)

        tagPose = None
        base_time = time.time()
        while tagPose == None and (time.time() - base_time) < CALIBRATE_TAG_WAIT_TIMEOUT:
            tagPose = vision_env.get_arg_tag(tagId)
            rospy.sleep(0.1)

        if tagPose == None:
            return CalibrateRobotToCameraResponse(status=False)

        # pre-process poses into tfs
        tagPos, tagRot = self._pose_msg_to_tf(tagPose)
        eePos, eeRot = self._pose_msg_to_tf(request.ee_pose)
        gtaPos, gtaRot = self._pose_msg_to_tf(request.tag_grip_tf)

        # compute transform between base_link and map
        position = [0 + eePos[i] + gtaPos[i] + tagPos[i] for i in range(0,len(eePos))]
        rotation = quaternion_multiply(quaternion_multiply(quaternion_multiply([0,0,0,1],eeRot),[0,0.707,0,0.707]),quaternion_inverse(tagRot))
        #TODO get the orientation down as right now it is not correct
        #self._update_baselink_map_tf(position,DEFAULT_TF['orientation'])

        print 'result', position, rotation

        return CalibrateRobotToCameraResponse(status=True)

    def _get_state(self, request):
        return GetEnvironmentState(
            grasped_task_objects=task_env.get_grasped_ids(),
            all_task_objects=task_env.get_all_task_ids(),
            all_vision_objects=vision_env.get_vision_ids(),
            all_ar_tags=vision_env.get_ar_ids())


if __name__ == "__main__":
    calibrate_tag = rospy.get_param('~calibrate_ar_tag_id',None)
    env = Environment(calibrate_tag)
    while not rospy.is_shutdown():
        rospy.spin()

'''
ee base_link (-0.00014280138463369698, 0.7043176170517356, 0.03148795015642926) (0.7071123348779547, 0.7070964926275097, -0.0021216872712473476, 0.0014813576356059509)
ee map [-0.06534820393737668, -0.03666618769995965, 0.2988520098005764] [ 0.67575675  0.70129839 -0.11709213  0.1944809 ]
'''
