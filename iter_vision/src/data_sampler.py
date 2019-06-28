#!/usr/bin/env python

'''
Data Sampler
Author: Curt Henrichs
Date: 6-28-19

Prints training data format for the vision model based on valid blocks.

Format provided is:

    '[<ratio>,<primary_axis>,<rotation>],'

'''

import rospy
from iter_vision.msg import BlockPose2D, BlockPose2DArray


def print_cb(message):
    print '----------------'
    for b in message.blocks:
        ratio = b.length / b.width
        primary_axis = b.length
        primary_rotation = b.pose.theta

        print '[{0}, {1}, {2}],'.format(ratio, primary_axis, primary_rotation)


if __name__ == "__main__":
    rospy.init_node('data_sampler')
    bp2_sub = rospy.Subscriber("/block_pose/pose_2d", BlockPose2DArray, print_cb)
    rospy.spin()
