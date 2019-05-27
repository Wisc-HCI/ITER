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


AREA_FILTER = (600,5000)

BIG_BLOCK_R = (9,11)
SML_BLOCK_R = (4,6)


class BlockVision:

    def __init__(self):
        self.pose_pub = rospy.Publisher("/block_vision/poses",BlockPoseArray, queue_size=1)
        self.img_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self._image_cb, queue_size=1)
        self.img_c_pub = rospy.Publisher("/block_vision/image/compressed",CompressedImage, queue_size=1)
        self.img_f_pub = rospy.Publisher("/block_vision/filtered/compressed",CompressedImage, queue_size=1)

    def _image_cb(self,imageMsg):

        # Load in image
        np_arr = np.fromstring(imageMsg.data, np.uint8)
        original_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Process image
        filtered_img = self._img_filter(original_img)

        # Publish image after filtering
        imgFMsg = CompressedImage()
        imgFMsg.header.stamp = rospy.Time.now()
        imgFMsg.format = "jpeg"
        imgFMsg.data = np.array(cv2.imencode('.jpg',filtered_img)[1]).tostring()
        self.img_f_pub.publish(imgFMsg)

        # Capture contours in image
        _0, contours, _1 = cv2.findContours(filtered_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through contours
        poses = []
        count = 0
        final_img = original_img.copy()
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
            if not (area >= AREA_FILTER[0] and area <= AREA_FILTER[1]):
                continue

            # Update output image
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            final_img = cv2.drawContours(final_img,[box],0,(0,255,0),2)

            # Classify
            type = BlockPose.UNKNOWN
            ratio = max((rect[1][0] / rect[1][1]),(rect[1][1] / rect[1][0]))
            if ratio >= BIG_BLOCK_R[0] and ratio <= BIG_BLOCK_R[1]:
                type = BlockPose.LARGE
            elif ratio >= SML_BLOCK_R[0] and ratio <= SML_BLOCK_R[1]:
                type = BlockPose.SMALL
            #print type

            # Generate pose information
            cx = rect[0][0]
            cy = rect[0][1]
            rotation = rect[2]
            if len(box) > 0:
                x_min = x_max = box[0][0]
                y_min = y_max = box[0][1]
                for b in box:
                    if b[0] < x_min:
                        x_min = b[0]
                    if b[1] < y_min:
                        y_min = b[1]
                    if b[0] > x_max:
                        x_max = b[0]
                    if b[1] > y_max:
                        y_max = b[1]
                x_ax = x_max - x_min
                y_ax = y_max - y_min
                rotation += 90 if y_ax > x_ax else 0

            # Package message
            block = BlockPose()
            block.pose = Pose2D(x=cx,y=cy,theta=rotation)
            block.type = type
            block.id = count
            poses.append(block)

            # Update ID counter
            count += 1

        # Publish object poses
        poseMsg = BlockPoseArray()
        poseMsg.blocks = poses
        self.pose_pub.publish(poseMsg)

        # Publish final image
        imgCMsg = CompressedImage()
        imgCMsg.header.stamp = rospy.Time.now()
        imgCMsg.format = "jpeg"
        imgCMsg.data = np.array(cv2.imencode('.jpg',final_img)[1]).tostring()
        self.img_c_pub.publish(imgCMsg)


    def _img_filter(self, original_image):

        # Apply an HSV filter to remove glare
        # Note that algorithm is assuming the objects are against a black background
        # otherwise another step to remove a solid background color is needed
        hsv = cv2.cvtColor(original_image,cv2.COLOR_BGR2HSV)
        thresh1 = cv2.inRange(hsv,(0,20,150),(179,255,255)) #0, 50, 50

        # Denoise
        kernel = np.ones((5,5),np.uint8)
        morphed = cv2.morphologyEx(thresh1,cv2.MORPH_OPEN,kernel)

        return morphed

if __name__ == "__main__":
    try:
        rospy.init_node("block_vision", anonymous=True)
        node = BlockVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
