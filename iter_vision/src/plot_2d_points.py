#!/usr/bin/env python

'''
Plot 2D Points Node
Author: Curt Henrichs
Date: 5-22-19

Plot 2D AR points found to image.

This node is used to verify correct identification of tags and to provide
insight into the 2D to 3D matching algorithm which will be used for blocks.
'''

import sys
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from iter_vision.msg import BlockPose2D, BlockPose2DArray
from ar_track_alvar_msgs.msg import AlvarMarker2D, AlvarMarker2DArray

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


class AR2DDraw:

    def __init__(self):
        self.img_sub = rospy.Subscriber("image/compressed", CompressedImage, self._image_cb, queue_size=1)
        self.img_pub = rospy.Publisher("plot_2d_points/image/compressed", CompressedImage, queue_size=1)
        self.art_sub = rospy.Subscriber("cam_pose_marker", AlvarMarker2DArray, self._art_cb, queue_size=5)
        self.blk_sub = rospy.Subscriber("block_pose/pose_2d", BlockPose2DArray, self._blk_cb, queue_size=5)

        self._markers = []
        self._blocks = []

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
            color = (255,0,0)
            image = cv2.drawContours(image,[box],0,color,2)
            textPos = (int(rect[0][0]+rect[1][0]),int(rect[0][1]-rect[1][1]/2))
            cv2.putText(image,'{0}'.format(m.id),textPos,cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)

        # Update image with block info
        for b in self._blocks:

            rect = ((b.pose.x,b.pose.y),(25,25),b.pose.theta)
            #   rect[0] #.center (x,y) 'float'
            #   rect[1] #.size (x,y) 'float'
            #   rect[2] #.angle 'float'

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            color = (0,0,255)
            image = cv2.drawContours(image,[box],0,color,2)
            textPos = (int(rect[0][0]+rect[1][0]),int(rect[0][1]-rect[1][1]/2))
            cv2.putText(image,'{0}:{1}'.format(b.id,b.type),textPos,cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)

        # Write output image
        imgMsg = CompressedImage()
        imgMsg.header.stamp = rospy.Time.now()
        imgMsg.format = "jpeg"
        imgMsg.data = np.array(cv2.imencode('.jpg',image)[1]).tostring()
        self.img_pub.publish(imgMsg)

    def _art_cb(self,message):
        self._markers = message.markers

    def _blk_cb(self,message):
        self._blocks = message.blocks


if __name__ == "__main__":
    try:
        rospy.init_node("ar_2d_image", anonymous=True)
        node = AR2DDraw()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
