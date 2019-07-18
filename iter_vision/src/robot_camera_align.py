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

import os
import tf
import yaml
import time
import math
import rospy
import numpy as np

from iter_vision.msg import CalibrationTf
from geometry_msgs.msg import Pose, Vector3, Quaternion
from iter_vision.srv import GetTagPose, GetTagPoseResponse
from iter_vision.srv import SetCalibrationTfs, SetCalibrationTfsResponse
from averaging_quaternions.averageQuaternions import averageQuaternions


CALIBRATION_FILEPATH = os.path.join(os.path.dirname(__file__),'config/camera_robot_calibration.yaml')


class RobotCameraAlignment:

    def __init__(self,reference_frame):
        self._load_calibration_file()

        self._reference_frame = reference_frame
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._get_tag_pose_srv = rospy.Service('robot_camera_align/get_tag_pose',GetTagPose, self._tf_pose_cb)
        self._calibrate_tfs = rospy.Service('robot_camera_align/set_tfs',SetCalibrationTfs, self._set_tf_cb)

    def _load_calibration_file(self):
        fin = open(CALIBRATION_FILEPATH,'r')
        tf_data = yaml.safe_load(fin)
        fin.close()
        self._tfs = {tf["child_frame"]: self._pack_tf(tf) for tf in tf_data}

    def _pack_tf(self, dct):
        msg = CalibrationTf()
        msg.child_frame = dct['child_frame']
        msg.parent_frame = dct['parent_frame']
        msg.position = dct['position']
        msg.rotation = dct['rotation']
        return msg

    def _store_calibration_file(self):
        tf_data = [self._unpack_tf(self._tfs[key]) for key in self._tfs.keys()]
        fout = open(CALIBRATION_FILEPATH,'w+')
        yaml.dump(tf_data, fout, default_flow_style=False)
        fout.close()

    def _unpack_tf(self, msg):
        dct = {
            'child_frame': msg.child_frame,
            'parent_frame': msg.parent_frame,
            'position': msg.position,
            'rotation': msg.rotation
        }
        return dct

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
        while (time.time() - start_time) <= request.duration:
            status, pos, rot = self._tf_lookup(request.tag_frame_id)
            if status:
                pos_list.append(pos)
                q = [rot[3],rot[0],rot[1],rot[2]]
                q_list.append(q)

        if len(pos_list) < 1 or len(rot_list) < 1:
            return GetTagPoseResponse(position=[0,0,0],rotation=[0,0,0,1],status=False)

        # Average calibrated tag pose
        pl = len(pos_list)
        px = sum([p[0] for p in pos_list]) / pl
        py = sum([p[1] for p in pos_list]) / pl
        pz = sum([p[2] for p in pos_list]) / pl
        pos = (px,py,pz)

        q = averageQuaternions(q_list)
        rot = [q[1],q[2],q[3],q[0]]

        # Generate tag pose
        return GetTagPoseResponse(position=pos,rotation=rot,status=True)

    def _set_tf_cb(self, request):
        for tf in request.tfs:
            self._tfs[tf.child_frame] = tf
        self._store_calibration_file()
        return True

    def spin(self):
        while not rospy.is_shutdown():
            self._refresh_tfs()
            rospy.sleep(0.5)

    def _refresh_tfs(self):
        for key in self._tfs.keys():
            point = self._tfs[key]
            self._tf_broadcaster.sendTransform(point.position,point.rotation,
                                               rospy.Time.now(),point.child_frame,
                                               point.parent_frame)


if __name__ == "__main__":
    try:
        rospy.init_node("robot_cam_align", anonymous=True)
        reference_frame = rospy.get_param('reference_frame','map')
        node = RobotCameraAlignment(reference_frame)
        node.spin()
    except rospy.ROSInterruptException:
        pass
