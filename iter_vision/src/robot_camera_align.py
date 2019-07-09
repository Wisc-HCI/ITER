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
import time
import math
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Vector3, Quaternion
from iter_vision.srv import GetTagPose, GetTagPoseResponse
from averaging_quaternions.averageQuaternions import averageQuaternions


CALIBRATION_DURATION = 5


class RobotCameraAlignment:

    def __init__(self,reference_frame):
        self._reference_frame = reference_frame
        self._etr = Pose(position=Vector3(x=0,y=0,z=0),orientation=Quaternion(x=0,y=0,z=0,w=1))

        self._get_tag_pose_srv = rospy.Service('/robot_camera_align/get_tag_pose',GetTagPose, self._tf_pose_cb)

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

    def _tf_pose_cb(self, request):
        pos_list = []
        q_list = []

        # Sample AR tag pose
        start_time = time.time()
        while (time.time() - start_time) <= CALIBRATION_DURATION:
            status, pos, rot = self._tf_lookup(request.tag_frame_id)
            if status:
                pos_list.append(pos)
                q = [rot[3],rot[0],rot[1],rot[2]]
                q_list.append(q)

        if len(pos_list) < 1 or len(rot_list) < 1:
            return GetTagPoseResponse(pose=Pose(),status=False)

        # Average calibrated tag pose
        pl = len(pos_list)
        px = sum([p[0] for p in pos_list]) / pl
        py = sum([p[1] for p in pos_list]) / pl
        pz = sum([p[2] for p in pos_list]) / pl
        pos = (px,py,pz)

        q = averageQuaternions(q_list)
        rot = [q[1],q[2],q[3],q[0]]

        # Generate tag pose
        pose = Pose()
        pose.position = Vector3(x=pos[0],y=pos[1],z=pos[2])
        pose.orientation = Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3])
        return GetTagPoseResponse(pose=pose,status=True)


if __name__ == "__main__":
    try:
        rospy.init_node("robot_cam_align", anonymous=True)
        reference_frame = rospy.get_param('reference_frame','usb_cam')
        node = RobotCameraAlignment(reference_frame)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
