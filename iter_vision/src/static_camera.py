#!/usr/bin/env python

'''
Static Camera Node
Author: Curt Henrichs
Date: 5-22-19

Provides a static image source from file provided.

This node acts as a fake camera for testing / debugging the vision system.

Parameters:
    - ~publish_rate
    - ~image_filepath
    - ~camera_ns

Publishers:
    - <namespace>/image_raw
    - <namespace>/image_raw/compressed
    - <namespace>/camera_info
'''

import os
import cv2
import sys
import yaml
import rospy
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import CameraInfo, CompressedImage

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


CALIBRATION_FILEPATH = os.path.join(os.path.dirname(__file__),'config/camera_calibration.yaml')


class StaticImageCam:

    def __init__(self, pub_rate, image_filepath, ns):
        self._pub_rate = pub_rate

        self.img_c_pub = rospy.Publisher(ns + "/image_raw/compressed",CompressedImage, queue_size=1)
        self.cam_pub = rospy.Publisher(ns + "/camera_info",CameraInfo,queue_size=1)

        fin = open(CALIBRATION_FILEPATH,'r')
        camData = yaml.safe_load(fin)
        fin.close()

        self._camMsg = CameraInfo()
        self._camMsg.header.frame_id = camData['camera_name']
        self._camMsg.height = camData['image_height']
        self._camMsg.width = camData['image_width']
        self._camMsg.distortion_model = camData['distortion_model']
        self._camMsg.D = camData['distortion_coefficients']['data']
        self._camMsg.K = camData['camera_matrix']['data']
        self._camMsg.R = camData['rectification_matrix']['data']
        self._camMsg.P = camData['projection_matrix']['data']

        image = cv2.imread(image_filepath,cv2.IMREAD_COLOR)
        self._imgCMsg = CompressedImage()
        self._imgCMsg.header.frame_id = camData['camera_name']
        self._imgCMsg.format = "jpeg"
        self._imgCMsg.data = np.array(cv2.imencode('.jpg',image)[1]).tostring()

    def spin(self):
        while not rospy.is_shutdown():
            time = rospy.Time.now()
            self._publish_cam_info(time)
            self._publish_image(time)
            rospy.sleep(1/self._pub_rate)

    def _publish_cam_info(self,time):
        self._camMsg.header.stamp = time
        self.cam_pub.publish(self._camMsg)

    def _publish_image(self,time):
        self._imgCMsg.header.stamp = time
        self.img_c_pub.publish(self._imgCMsg)


if __name__ == "__main__":
    try:
        rospy.init_node("static_camera", anonymous=True)

        pubRate = rospy.get_param('~publish_rate',1)
        imageFilePath = rospy.get_param('~image_filepath',None)
        cameraNs = rospy.get_param('~camera_ns','static_cam')

        node = StaticImageCam(pubRate,imageFilePath,cameraNs)
        node.spin()
    except rospy.ROSInterruptException:
        pass
