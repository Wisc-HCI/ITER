#!/usr/bin/env python

'''
Block Pose Node
Author: Curt Henrichs
Date: 5-22-19

Converts from 2D BlockPose coordinates to 3D BlockPose coordinates.

Using the AR region identified, generate a mapping from 2D image coordinates to
3D world coordinates relative to the camera.

Publishes these coordinates as a BlockPoseArray and updates TF entries for blocks
identified.
'''

import tf
import sys
import math
import rospy
import numpy as np

from std_msgs.msg import Header
from iter_vision.msg import BlockPose, BlockPoseArray
from geometry_msgs.msg import PoseArray, Pose, Pose2D, Vector3, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker2D, AlvarMarker2DArray
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkerArray


ROTATION_CONSTANT = 1


class BlockPose:

    def __init__(self):
        self.ar3_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkerArray, self._ar3_cb, queue_size=5)
        self.ar2_sub = rospy.Subscriber("cam_pose_marker", AlvarMarker2DArray, self._ar2_cb, queue_size=5)
        self.bp_sub = rospy.Subscriber("/block_vision/poses", BlockPoseArray, self._bp_cb, queue_size=5)
        self.pose_pub = rospy.Publisher("/block_poses", PoseArray, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

        self._ar3_markers = []
        self._ar2_markers = []
        self._new_ar_flag = True
        self._computed_position_transform = None
        self._plane_orientation = None
        self._image_orienation = 0

    def _ar2_cb(self, message):
        self._ar2_markers = message.markers
        self._new_ar_flag = True

    def _ar3_cb(self, message):
        self._ar3_markers = message.markers
        self._new_ar_flag = True

    def _pub_tf_poses(self,poses):
        for i in range(0,len(poses)):
            p = poses[i]
            position = [p.position.x,p.position.y,p.position.z]
            orientation = [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
            self.tf_br.sendTransform(position,orientation,rospy.Time.now(),'block_{0}'.format(i),'usb_cam')

    def _bp_cb(self, message):
        p3_array = []

        if self._new_ar_flag:
            # Find A input matrix and Y output matrix
            # Also compute the standard cam to plane orientation
            r3 = [0,0,0]
            r2 = 0
            count = 0
            A = None
            Y = None
            for m3 in self._ar3_markers:
                for m2 in self._ar2_markers:
                    if m3.id == m2.id:
                        if count == 0:
                            A, Y = self._point_position_eqs(m2.pose.x,m2.pose.y,m3.pose.pose.position.x,m3.pose.pose.position.y,m3.pose.pose.position.z)
                            r2 = m2.pose.theta
                            r3 = [m3.pose.pose.orientation.x,m3.pose.pose.orientation.y,m3.pose.pose.orientation.z,m3.pose.pose.orientation.w]
                        else:
                            a, y = self._point_position_eqs(m2.pose.x,m2.pose.y,m3.pose.pose.position.x,m3.pose.pose.position.y,m3.pose.pose.position.z)
                            A = np.append(A,a,axis=0)
                            Y = np.append(Y,y,axis=0)
                        count += 1

            # At least minimum eqs matrix for psuedo-inverse
            if count >= 3:
                invA = np.linalg.pinv(A)
                self._computed_position_transform = invA * Y
                self._image_orienation = r2
                self._plane_orientation = r3

        # Apply transform from 2D space to 3D space on blocks
        if not (self._computed_position_transform is None or self._plane_orientation is None):
            for b2 in message.blocks:
                A, _ = self._point_position_eqs(b2.pose.x,b2.pose.y)
                projection = A * self._computed_position_transform

                position = Vector3(x=projection[0,0],
                                   y=projection[1,0],
                                   z=projection[2,0])

                rz =  ROTATION_CONSTANT * (b2.pose.theta / 180 * math.pi) - self._image_orienation
                q = tf.transformations.quaternion_from_euler(0,0,rz)
                x,y,z,w = tf.transformations.quaternion_multiply(self._plane_orientation, q)
                orientation = Quaternion(x=x,y=y,z=z,w=w)

                p3 = Pose(position=position,orientation=orientation)
                p3_array.append(p3)

        # Publish array of poses
        self._pub_tf_poses(p3_array)
        self.pose_pub.publish(PoseArray(header=Header(),poses=p3_array))

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
        node = BlockPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
