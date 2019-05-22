#!/usr/bin/env python

'''
AR 2D Image Node
Author: Curt Henrichs
Date: 5-22-19

Plot 2D AR points found to image.

This node is used to verify correct identification of tags and to provide
insight into the 2D to 3D matching algorithm which will be used for blocks.
'''

import tf
import sys
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker2D, AlvarMarker2DArray

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


class AR2DDraw:

    def __init__(self):
        self.img_sub = rospy.Subscriber("image/compressed", CompressedImage, self._image_cb, queue_size=1)
        self.img_pub = rospy.Publisher("ar_2d/image/compressed", CompressedImage, queue_size=1)
        self.ar_sub = rospy.Subscriber("cam_pose_marker", AlvarMarker2DArray, self._ar_cb, queue_size=5)
        self._markers = []

    def _image_cb(self, imageMsg):

        # Load in image
        np_arr = np.fromstring(imageMsg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, channels = image.shape

        # Update image with tag info
        for m in self._markers:

            rect = ((m.pose.x,m.pose.y),(25,25),m.pose.theta)
            #   rect[0] #.center (x,y) 'float'
            #   rect[1] #.size (x,y) 'float'
            #   rect[2] #.angle 'float'

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            image = cv2.drawContours(image,[box],0,(255,0,0),2)

        # Write output image
        imgMsg = CompressedImage()
        imgMsg.header.stamp = rospy.Time.now()
        imgMsg.format = "jpeg"
        imgMsg.data = np.array(cv2.imencode('.jpg',image)[1]).tostring()
        self.img_pub.publish(imgMsg)

    def _ar_cb(self,message):
        self._markers = message.markers


if __name__ == "__main__":
    try:
        rospy.init_node("ar_2d_image", anonymous=True)
        node = AR2DDraw()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
