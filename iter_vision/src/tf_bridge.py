#!/usr/bin/env python

'''
TF Bridge Node
Author: Curt Henrichs
Date: 7-12-19

TF Bridge maps from internal tf space for vision pipeline into the global tf tree.

Due to design reasons, there is are times where an internal tf tree needs to
be maintained under seperate frame naming (typically due to how nodes were
implemented). This node then acts as a bridge from that tree into the global
tf tree.

Currently this node does not filter messages. Future work to provide black- and
white-listing.
'''
#TODO whitelisting and blacklisting

import tf
import rospy
import tf2_ros

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TFBridge:

    def __init__(self,tf_ns):
        self._tf_ns = tf_ns

        self._tf_pub = rospy.Publisher('/tf',TFMessage,queue_size=5)
        self._tf_static_pub = tf2_ros.StaticTransformBroadcaster()

        self._tf_sub = rospy.Subscriber('tf',TFMessage,self._tf_cb)
        self._tf_static_sub = rospy.Subscriber('tf_static',TFMessage,self._tf_static_cb)

    def _tf_cb(self, message):
        for t in message.transforms:
            t.header.frame_id = self._tf_ns + t.header.frame_id
            t.child_frame_id = self._tf_ns + t.child_frame_id

        self._tf_pub.publish(message)

    def _tf_static_cb(self, message):
        for t in message.transforms:
            t.header.frame_id = self._tf_ns + t.header.frame_id
            t.child_frame_id = self._tf_ns + t.child_frame_id

        self._tf_static_pub.sendTransform(message.transforms)


if __name__ == "__main__":
    rospy.init_node('tf_bridge')

    tf_ns = rospy.get_param('~tf_ns','')

    node = TFBridge(tf_ns)

    rospy.spin()
