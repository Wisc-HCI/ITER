#!/usr/bin/env python

'''
BlockVision Node
Author: Curt Henrichs
Date: 5-22-19

Provides ITER with ability to capture blocks from a vision

This node requires installation of opencv and its python wrapper. Additionally,
this node requires the usb_cam node be running in order to provide an image
stream to this node.

See tutorial
https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki
https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Installation
for information on how to work through OpenCV

The algorithm behind this module is rather simple (and brittle), future work to
enhance / extend this is necessary. Currently it filters image in HSV space
(removing low saturation and low value). Then applying erroision and dialation
to clean up the binary image. The end result is a set of blobs which are fit to
a min area rectangle where the area is filtered to be within the expected range
of blocks (thereby eliminating small noise blobs and large invalid objects).

After detecting the blocks, a ration between length and width is used to
determine if the block is the large or small block. These resulting detected
blocks are then published as a 2D pose in 3D space. Where centroid x,y is provided
and z is 0 and where orientation is the Euler angle around the z-axis.
'''

import sys
import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage
from iter_vision.msg import BlockPose, BlockPoseArray

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


BIG_BLOCK_A = (100000000,200000000)
SML_BLOCK_A = (1000,5000)

BIG_BLOCK_R = (7,8)
SML_BLOCK_R = (4,5)


class BlockVision:

    def __init__(self):
        self.pose_pub = rospy.Publisher("/block_vision/poses",BlockPoseArray, queue_size=1)
        self.img_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self._image_cb, queue_size=1)
        self.img_pub = rospy.Publisher("/block_vision/image/compressed",CompressedImage, queue_size=1)


    def _image_cb(self,imageMsg):

        # Load in image
        np_arr = np.fromstring(imageMsg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, channels = image_np.shape

        # Apply an HSV filter to remove glare
        # Note that algorithm is assuming the objects are against a black background
        # otherwise another step to remove a solid background color is needed
        hsv = cv2.cvtColor(image_np,cv2.COLOR_BGR2HSV)
        thresh1 = cv2.inRange(hsv,(0,50,50),(179,255,255))

        # Denoise
        kernel = np.ones((5,5),np.uint8)
        morphed = cv2.morphologyEx(thresh1,cv2.MORPH_OPEN,kernel)

        # Capture contours in image
        _0, contours, _1 = cv2.findContours(morphed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through contours
        poses = []
        final_img = image_np.copy()
        for cnt in contours:

            # Fit rectangle to contour
            #   rect[0] #.center (x,y) 'float'
            #   rect[1] #.size (x,y) 'float'
            #   rect[2] #.angle 'float'
            rect = cv2.minAreaRect(cnt)

            # Filter by known attributes of blocks
            #   check object area
            #   check dimension ratio
            area = rect[1][0] * rect[1][1]
            #print area
            if not ((area >= BIG_BLOCK_A[0] and area <= BIG_BLOCK_A[1]) or (area >= SML_BLOCK_A[0] and area <= SML_BLOCK_A[1])):
                continue

            type = BlockPose.UNKNOWN
            '''
            ratio = max((rect[1][0] / rect[1][1]),(rect[1][1] / rect[1][0]))
            if ratio >= BIG_BLOCK_R[0] and ratio <= BIG_BLOCK_R[1]:
                type = BlockPose.LARGE
            elif ratio >= SML_BLOCK_R[0] and ratio <= SML_BLOCK_R[1]:
                type = BlockPose.SMALL
            else:
                continue
            '''

            # Generate pose information
            cx = rect[0][0]
            cy = rect[0][1]
            block = BlockPose()
            block.pose = Pose2D(x=cx,y=cy,theta=rect[2])
            block.type = type
            poses.append(block)

            # Update output image
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            final_img = cv2.drawContours(final_img,[box],0,(0,255,0),2)

        # Publish object poses
        poseMsg = BlockPoseArray()
        poseMsg.blocks = poses
        self.pose_pub.publish(poseMsg)

        # Publish final image
        imgMsg = CompressedImage()
        imgMsg.header.stamp = rospy.Time.now()
        imgMsg.format = "jpeg"
        imgMsg.data = np.array(cv2.imencode('.jpg',final_img)[1]).tostring()
        self.img_pub.publish(imgMsg)


if __name__ == "__main__":
    try:
        rospy.init_node("block_vision", anonymous=True)
        node = BlockVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
