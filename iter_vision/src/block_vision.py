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

# TODO large blocks not being detected
# HSV needs to be adjusted for lighting

import sys
import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage
from iter_vision.msg import BlockPose2D, BlockPose2DArray
from iter_vision.srv import ColorSelect, ColorSelectRequest, ColorSelectResponse

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


AREA_FILTER = (600,4000)

SML_BLOCK_AREA = (,)
BIG_BLOCK_AREA = (,)

BIG_BLOCK_RATIO = (3.5,7.5)
SML_BLOCK_RATIO = (2,4)


class BlockVision:

    def __init__(self):
        self._min_hue = ColorSelectRequest.MIN_HUE_VALUE
        self._max_hue = ColorSelectRequest.MAX_HUE_VALUE

        self.pose_pub = rospy.Publisher("/block_vision/poses",BlockPose2DArray, queue_size=1)
        self.img_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self._image_cb, queue_size=1)
        self.img_c_pub = rospy.Publisher("/block_vision/image/compressed",CompressedImage, queue_size=1)
        self.img_f_pub = rospy.Publisher("/block_vision/filtered/compressed",CompressedImage, queue_size=1)

        self.color_select_srv = rospy.Service("/block_vision/color_select",ColorSelect,self._color_select_cb)

    def _color_select_cb(self,response):
        status = True

        if self._min_hue < ColorSelectRequest.MIN_HUE_VALUE:
            status = False
        elif seld._max_hue > ColorSelectRequest.MAX_HUE_VALUE:
            status = False
        else:
            self._min_hue = response.min_hue
            self._max_hue = response.max_hue

        return ColorSelectResponse(status=status)

    def _image_cb(self,imageMsg):

        # Load in image
        np_arr = np.fromstring(imageMsg.data, np.uint8)
        original_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Process image
        canny_mask = self._canny_filter(original_img)
        #filtered_img = canny_mask
        temp_img = cv2.bitwise_and(original_img,original_img,mask=canny_mask)
        filtered_img = self._hsv_vision_image_filter(temp_img)

        # Publish image after filtering
        imgFMsg = CompressedImage()
        imgFMsg.header.stamp = rospy.Time.now()
        imgFMsg.format = "jpeg"
        imgFMsg.data = np.array(cv2.imencode('.jpg',filtered_img)[1]).tostring()
        self.img_f_pub.publish(imgFMsg)

        # detect bloks from image
        final_img, poses = self._contour_based_detection(original_img,filtered_img)

        #print '----------------'
        #print poses

        # Publish object poses
        poseMsg = BlockPose2DArray()
        poseMsg.blocks = poses
        self.pose_pub.publish(poseMsg)

        # Publish final image
        imgCMsg = CompressedImage()
        imgCMsg.header.stamp = rospy.Time.now()
        imgCMsg.format = "jpeg"
        imgCMsg.data = np.array(cv2.imencode('.jpg',final_img)[1]).tostring()
        self.img_c_pub.publish(imgCMsg)

    def _hsv_vision_image_filter(self, original_img):
        # Apply an HSV filter to remove glare
        # Note that algorithm is assuming the objects are against a black background
        # otherwise another step to remove a solid background color is needed
        hsv = cv2.cvtColor(original_img,cv2.COLOR_BGR2HSV)
        thresh1 = cv2.inRange(hsv,(self._min_hue,10,100),(self._max_hue,255,255)) #0, 50, 50

        # Denoise
        kernel_5 = np.ones((5,5),np.uint8)
        kernel_3 = np.ones((3,3),np.uint8)
        erosion = cv2.erode(thresh1,kernel_5,iterations=1)
        dilation = cv2.dilate(erosion,kernel_5,iterations=2)
        erosion = cv2.erode(dilation,kernel_5,iterations=2)
        dilation = cv2.dilate(erosion,kernel_5,iterations=1)
        erosion = cv2.erode(dilation,kernel_5,iterations=2)
        dilation = cv2.dilate(erosion,kernel_3,iterations=2)
        return dilation

    def _canny_filter(self, original_img):
        gray = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(original_img,100,200)

        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(edges,kernel,iterations=5)
        erosion = cv2.erode(dilation,kernel,iterations=4)
        return erosion

    def _contour_based_detection(self,original_img,filtered_img):
        # Capture contours in image
        _0, contours, _1 = cv2.findContours(filtered_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        print '----------------'

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
            if not (area >= AREA_FILTER[0] and area <= AREA_FILTER[1]):
                continue

            # Update output image
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            final_img = cv2.drawContours(final_img,[box],0,(0,255,0),2)

            # Classify
            type = BlockPose2D.UNKNOWN
            ratio = max((rect[1][0] / rect[1][1]),(rect[1][1] / rect[1][0]))
            if ratio >= BIG_BLOCK_RATIO[0] and ratio < BIG_BLOCK_RATIO[1]:
                type = BlockPose2D.LARGE
            elif ratio >= SML_BLOCK_RATIO[0] and ratio < SML_BLOCK_RATIO[1]:
                type = BlockPose2D.SMALL

            print ratio

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
            block = BlockPose2D()
            block.pose = Pose2D(x=cx,y=cy,theta=rotation)
            block.type = type
            block.id = count
            poses.append(block)

            # Update ID counter
            count += 1

        return final_img, poses


if __name__ == "__main__":
    try:
        rospy.init_node("block_vision", anonymous=True)
        node = BlockVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
