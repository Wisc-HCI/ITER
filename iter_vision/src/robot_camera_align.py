#!/usr/bin/env python

'''
Robot Camera Align Node
Author: Curt Henrichs
Date: 5-22-19

Uses an AR tag on the robot gripper to calibrate relationship between camera TF
and base_link and end_effector TF.

This node provides a service to search for a given AR tag TF, retrieve the pose
relative to the camera. Additionally, provides a modified pose to take the
AR tag to end-effector pose relationship into account. Note: this relationship
must be updated via a service before use.

Services provided:
    - get_tag_pose
    - get_ee_pose
    - set_ee_tag_transform
    - get_ee_tag_transform

Parameters:
    - reference_frame
'''

import tf
import math
import rospy
import numpy as np

from iter_vision.srv import GetEEPose, GetEEPoseResponse
from iter_vision.srv import GetTagPose, GetTagPoseResponse
from iter_vision.srv import GetTagToEETransform, GetTagToEETransformResponse
from iter_vision.srv import SetTagToEETransform, SetTagToEETransformResponse
from geometry_msgs.msg import Pose, Vector3, Quaternion

class RobotCameraAlignment:

    def __init__(self,reference_frame):
        self._reference_frame = reference_frame
        self._etr = Pose(position=Vector3(x=0,y=0,z=0),orientation=Quaternion(x=0,y=0,z=0,w=1))

        self._get_tag_pose_srv = rospy.Service('/robot_camera_align/get_tag_pose',GetTagPose, self._tf_pose_cb)
        self._get_ee_pose_srv = rospy.Service('/robot_camera_align/get_ee_pose',GetEEPose, self._ee_pose_cb)
        self._get_tag_to_ee_tf_srv = rospy.Service('/robot_camera_align/get_tag_to_ee_transform', GetTagToEETransform, self._get_ee_tag_r_cb)
        self._set_tag_to_ee_tf_srv = rospy.Service('/robot_camera_align/set_tag_to_ee_transform', SetTagToEETransform, self._set_ee_tag_r_cb)

        self._tf_listener = tf.TransformListener()

    def _tf_lookup(self,tag_id):
        status = True
        try:
            (translation,rotation) = self._tf_listener.lookupTransform(self._reference_frame,tag_id, rospy.Time(0))
        except (tf.lookupException, tf.ConnectivityException, tf.ExtrapolationException):
            status = False
            translation = None
            rotation = None
        return status, translation, rotation

    def _extract_ee_transform(self):
        translation = [self._etr.position.x,self._etr.position.y,self._etr.position.z]
        rotation = [self._etr.orientation.x,self._etr.orientation.y,self._etr.orientation.z,self._etr.orientation.w]
        return translation, rotation

    def _tf_pose_cb(self, request):
        status, translation, rotation = self._tf_lookup(request.tag_frame_id)

        pose = Pose()
        if status:
            pose.position = translation
            pose.orientation = rotation

        return GetTagPoseResponse(pose=pose,status=status)

    def _ee_pose_cb(self, request):
        status, ar_trans, ar_rot = self._tf_lookup(request.tag_frame_id)
        ee_trans, ee_rot = self._extract_ee_transform()

        pose = Pose()
        if status:
            (px,py,pz) = np.add(np.matrix(ar_trans),np.matrix(ee_trans))
            (ox,oy,oz,ow) = tf.transformations.quaternion_multiply(ar_rot,ee_rot).tolist()
            pose.position = Vector3(x=px,y=py,z=pz)
            pose.orientation = Quaternion(x=ox,y=oy,z=oz,w=ow)

        return GetEEPoseResponse(pose=pose,status=status)

    def _set_ee_tag_r_cb(self, request):
        self._etr = request.transform
        return SetTagToEETransformResponse(True)

    def _get_ee_tag_r_cb(self, request):
        return GetTagToEETransformResponse(transform=self._etr)


if __name__ == "__main__":
    try:
        rospy.init_node("robot_cam_align", anonymous=True)
        reference_frame = rospy.get_param('reference_frame','usb_cam')
        node = RobotCameraAlignment(reference_frame)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
