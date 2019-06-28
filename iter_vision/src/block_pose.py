#!/usr/bin/env python

'''
Block Pose Node
Author: Curt Henrichs
Date: 5-22-19

Converts from 2D BlockPose coordinates to 3D BlockPose coordinates.

Using the AR region identified, generate a mapping from 2D image coordinates to
3D world coordinates relative to the camera.

Publishes these coordinates as a BlockPose3DArray and updates TF entries for blocks
identified.

Takes in rosparam for 'parent_frame_id' which defines parent to block pose transforms
Takes in rosparam for 'rotation_constant' which is partially dependent on camera to table pose relationship
    however for a roughly top-down view it is approximately 1.
Takes in rosparam for 'ar_exclusion_radius' which is a circle around the ar tag where blocks
    will be ignored. This is due to artifacts from AR tag being detected as a block.
'''

import tf
import math
import rospy
import random
import numpy as np

from shapely import geometry

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from iter_vision.msg import BlockPose2D, BlockPose2DArray
from iter_vision.msg import BlockPose3D, BlockPose3DArray
from geometry_msgs.msg import PoseArray, Pose, Pose2D, Vector3, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker2D, AlvarMarker2DArray
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkerArray


ROTATION_CONSTANT = 1
AR_RADIUS = 50

class BlockPoseNode:

    def __init__(self, parent_frame_id, rotation_constant,tag_id_list,ar_radius):
        self._parent_frame_id = parent_frame_id
        self._rotation_constant = rotation_constant
        self._tag_id_list = tag_id_list
        self._ar_radius = ar_radius

        self.cam_pub = rospy.Subscriber("/usb_cam/camera_info",CameraInfo, self._cam_info_cb, queue_size=1)
        self.ar3_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkerArray, self._ar3_cb, queue_size=5)
        self.ar2_sub = rospy.Subscriber("cam_pose_marker", AlvarMarker2DArray, self._ar2_cb, queue_size=5)
        self.bp_sub = rospy.Subscriber("/block_vision/poses", BlockPose2DArray, self._bp_cb, queue_size=5)
        self.bp2_pub = rospy.Publisher("/block_pose/pose_2d", BlockPose2DArray, queue_size=10)
        self.bp3_pub = rospy.Publisher("/block_pose/pose_3d", BlockPose3DArray, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

        self._ar3_markers = []
        self._ar2_markers = []
        self._new_ar_flag = True
        self._computed_position_transform = None
        self._plane_orientation = None
        self._image_orienation = 0
        self._img_height = 0
        self._img_width = 0

    def _cam_info_cb(self, message):
        self._img_height = message.height
        self._img_width = message.width

    def _ar2_cb(self, message):
        self._ar2_markers = message.markers
        self._new_ar_flag = True

    def _ar3_cb(self, message):
        self._ar3_markers = message.markers
        self._new_ar_flag = True

    def _pub_tf_poses(self,blocks):
        for b in blocks:
            position = [b.pose.position.x,b.pose.position.y,b.pose.position.z]
            orientation = [b.pose.orientation.x,b.pose.orientation.y,b.pose.orientation.z,b.pose.orientation.w]
            self.tf_br.sendTransform(position,orientation,b.header.stamp,b.header.frame_id,b.parent_frame_id)

    def _bp_cb(self, message):

        # if new AR data, calculate new block transforms
        if self._new_ar_flag:
            self._calculate_new_transform()

        b2_array = []
        b3_array = []
        if not (self._computed_position_transform is None or self._plane_orientation is None):

            # filter blocks to only those in region
            b2_array = self._filter_on_region(message.blocks)

            # Apply transform from 2D space to 3D space on blocks
            b3_array = self._apply_transform(b2_array)

        # Publish array of poses
        self._pub_tf_poses(b3_array)
        self.bp3_pub.publish(BlockPose3DArray(header=Header(),blocks=b3_array))
        self.bp2_pub.publish(BlockPose2DArray(header=Header(),blocks=b2_array))

    def _calculate_new_transform(self):
        # Find A input matrix and Y output matrix
        # Also compute the standard cam to plane orientation
        rotations = []
        count = 0
        A = None
        Y = None
        for m3 in self._ar3_markers:
            for m2 in self._ar2_markers:

                # if match and either in tag list or if not using tag list
                processTag = (m3.id == m2.id) and (self._tag_id_list is None or str(m2.id) in self._tag_id_list)
                if processTag:
                    if count == 0:
                        A, Y = self._point_position_eqs(m2.pose.x,m2.pose.y,m3.pose.pose.position.x,m3.pose.pose.position.y,m3.pose.pose.position.z)
                    else:
                        a, y = self._point_position_eqs(m2.pose.x,m2.pose.y,m3.pose.pose.position.x,m3.pose.pose.position.y,m3.pose.pose.position.z)
                        A = np.append(A,a,axis=0)
                        Y = np.append(Y,y,axis=0)

                    r2 = m2.pose.theta
                    r3 = [m3.pose.pose.orientation.x,m3.pose.pose.orientation.y,m3.pose.pose.orientation.z,m3.pose.pose.orientation.w]
                    rotations.append([r2,r3])

                    count += 1

        # At least minimum eqs matrix for psuedo-inverse
        if count >= 3:

            # calculate position transform
            invA = np.linalg.pinv(A)
            self._computed_position_transform = invA * Y

            # calculate orientation transform
            median = np.median([r[0] for r in rotations])
            r2 = rotations[0][0]
            r3 = rotations[0][1]
            for r in rotations:
                if abs(median - r[0]) < abs(median - r2):
                    r2 = r[0]
                    r3 = r[1]
            self._image_orienation = (2 * math.pi - r2 / 180.0 * math.pi)
            self._plane_orientation = r3

    def _apply_transform(self,b2_array):
        currentTime = rospy.Time.now()
        b3_array = []

        for b2 in b2_array:
            A, _ = self._point_position_eqs(b2.pose.x,b2.pose.y)
            projection = A * self._computed_position_transform

            position = Vector3(x=projection[0,0],
                               y=projection[1,0],
                               z=projection[2,0])

            raw_angle = (2 * math.pi - b2.pose.theta / 180.0 * math.pi)
            rz = self._rotation_constant * raw_angle - self._image_orienation
            q = tf.transformations.quaternion_from_euler(0,0,rz)
            x,y,z,w = tf.transformations.quaternion_multiply(self._plane_orientation, q)
            orientation = Quaternion(x=x,y=y,z=z,w=w)

            b3 = BlockPose3D()
            b3.header.stamp = currentTime
            b3.header.frame_id = 'block_{0}'.format(b2.id)
            b3.parent_frame_id = self._parent_frame_id
            b3.pose = Pose(position=position,orientation=orientation)
            b3.type = b2.type
            b3.id = b2.id
            b3.length = b2.length
            b3.width = b2.width
            b3_array.append(b3)
        return b3_array

    def _filter_on_region(self,b2_array):

        # define polygon from ar tags for regions
        poly = geometry.MultiPoint([(a2.pose.x,a2.pose.y) for a2 in self._ar2_markers]).convex_hull

        # iterate over blocks selecting only those with centroid within region
        filtered_array = []
        for b2 in b2_array:
            p2 = geometry.Point(b2.pose.x,b2.pose.y)

            in_region = poly.contains(p2)

            in_ar_circle = any([self._cartesian_distance((b2.pose.x,b2.pose.y),(a2.pose.x,a2.pose.y)) < self._ar_radius for a2 in self._ar2_markers])

            if in_region and not in_ar_circle:
                filtered_array.append(b2)

        return filtered_array

    def _cartesian_distance(self,p1,p2):
        return math.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))

    def _point_position_eqs(self,i,j,x=0,y=0,z=0):
        A = np.matrix([
            [i,j,0,0,0,0,1,0,0],
            [0,0,i,j,0,0,0,1,0],
            [0,0,0,0,i,j,0,0,1]
        ])
        Y = np.matrix([
            [x],
            [y],
            [z]
        ])
        return A, Y


if __name__ == "__main__":
    try:
        rospy.init_node("block_pose", anonymous=True)
        parent_frame_id = rospy.get_param("~parent_frame_id",'usb_cam')
        rotation_constant = rospy.get_param("~rotation_constant",ROTATION_CONSTANT)
        tag_id_list = rospy.get_param('~tag_id_list',None)
        ar_radius = rospy.get_param('~ar_exclusion_radius',AR_RADIUS)
        node = BlockPoseNode(parent_frame_id,rotation_constant,tag_id_list,ar_radius)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
