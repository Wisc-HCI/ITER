#!/usr/bin/env python
'''
BlockVision Node

Provides ITER with ability to capture blocks from a vision

This node requires installation of opencv and its python wrapper. Additionally,
this node requires the usb_cam node be running in order to provide an image
stream to this node.

See tutorial
https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki
https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Installation
for information on how to work through OpenCV
'''

import tf
import sys
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, Point, Quaternion
from iter_app.msgs import BlockPose, BlockPoseArray

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


BIG_BLOCK_A = (22000,24000)
SML_BLOCK_A = (10000,12000)

BIG_BLOCK_R = (7,8)
SML_BLOCK_R = (4,5)


class BlockVision:

    def __init__(self):
        self.pose_pub = rospy.Publisher("/block_vision/poses",BlockPoseArray, queue_size=1)
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,self._image_cb, queue_size=1)
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
            if not ((area >= BIG_BLOCK_A[0] and area <= BIG_BLOCK_A[1]) or (area >= SML_BLOCK_A[0] and area <= SML_BLOCK_A[1])):
                continue

            type = BlockPose.UNKNOWN
            ratio = max((rect[1][0] / rect[1][1]),(rect[1][1] / rect[1][0]))
            if ratio >= BIG_BLOCK_R[0] and ratio <= BIG_BLOCK_R[1]:
                type = BlockPose.LARGE
            elif ratio >= SML_BLOCK_R[0] and ratio <= SML_BLOCK_R[1]:
                type = BlockPose.SMALL
            else:
                continue

            # Generate pose information
            cx = (rect[0][0] - 0.5 * width) / width
            cy = (rect[0][1] - 0.5 * height) / height
            qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0,0,rect[2])

            block = BlockPose()
            block.pose = Pose(position=Point(x=cx,y=cy,z=0),orientation=Quaternion(x=qx,y=qy,z=qz,w=qw))
            block.type = type
            poses.append(block)

            # Update output image
            min_point = tuple([int(rect[0][i] - rect[1][i]/2) for i in range(0,2)])
            max_point = tuple([int(rect[0][i] + rect[1][i]/2) for i in range(0,2)])
            cv2.rectangle(final_img,min_point,max_point,(0,255,0),2)

        # Publish object poses
        poseMsg = BlockPoseArray()
        poseMsg.data = poses
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
