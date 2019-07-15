#!/usr/bin/env python

'''
Two Camera Agreement Node
Author: Curt Henrichs
Date: 7-12-19

Generates the final environment sense from multiple cameras.

Global TFs:
    'block_{0}'
    'ar_marker_{0}'
    '/map'
'''

import copy
import rospy

from iter_vision.msg import BlockPose3D, BlockPose3DArray
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkerArray
from iter_vision.srv import ColorSelect, ColorSelectRequest, ColorSelectResponse


class TwoCameraAgreement:

    def __init__(self):

        # Transform Interface
        self._calibration_tag = 4
        self._tfs = {
            'map_to_cam_1_map': {'position':(0,0,0),'orientation':(0,0,0,1)},
            'cam_1_map_to_agreement_point': {'position':(0,0,0),'orientation':(0,0,0,1)},
            'agreement_point_to_cam_2_map': {'position':(0,0,0),'orientation':(0,0,0,1)}
        }
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()

        # Camera Interface
        cam_list = ['cam_1','cam_2']
        self._cams = {}
        for cam in cam_list:
            self._tfs[cam + '_map'] = None

            self._cams[cam] = {}
            self._cams[cam]["color_slct_srv"] = rospy.ServiceProxy(cam + '/block_vision/color_select',ColorSelect)
            self._cams[cam]["ar3_sub"] = rospy.ServiceProxy(cam + '/ar_pose_marker'AlvarMarkerArray,lambda x: self._ar3_cb(cam,x))
            self._cams[cam]["bk3_sub"] = rospy.ServiceProxy(cam + '/block_pose/pose_3d',BlockPose3DArray,lambda x: self._bk3_cb(cam,x))
            self._cams[cam]["ar_poses"] = []
            self._cams[cam]["bk_poses"] = []

        # Global Interface
        self._ar3_pub = rospy.Publisher("ar_pose_marker",AlvarMarkerArray, queue_size=5)
        self._bk3_pub = rospy.Publisher("block_pose/pose_3d", BlockPose3DArray, queue_size=5)
        self._color_slct_srv = rospy.Service("block_vision/color_select",ColorSelect, self._color_slct_cb)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._process_transform_between_cams()
            self._process_ar_tag_agreement()
            self._process_block_agreement()
            rate.sleep()

    def _process_transform_between_cams(self):
        #Note: map -> cam_1_map -> calibration -> cam_2_map

        # Retrieve marker agreement_point
        try:
            (cam_1_tagPos, cam_1_tagRot) self._tf_listener.lookupTransform('cam_1_ar_marker_{}'.format(self._calibration_tag),'cam_1_map')
            (cam_2_tagPos, cam_2_tagRot) self._tf_listener.lookupTransform('cam_2_ar_marker_{}'.format(self._calibration_tag),'cam_2_map')

            self._tfs['cam_1_map_to_agreement_point']['position'] = cam_1_tagPos
            self._tfs['cam_1_map_to_agreement_point']['orientation'] = cam_1_tagRot
            self._tfs['agreement_point_to_cam_2_map']['position'] = cam_2_tagPos
            self._tfs['agreement_point_to_cam_2_map']['orientation'] = cam_2_tagRot

        except Exception, e:
            pass # Marker was not found in both

        # Pubish TF update
        _t = self._tfs['map_to_cam_1_map']
        self._tf_broadcaster.sendTransform(_t['position'],_t['orientation'],rospy.Time.now(),'cam_1_map','map')
        _t = self._tfs['cam_1_map_to_agreement_point']
        self._tf_broadcaster.sendTransform(_t['position'],_t['orientation'],rospy.Time.now(),'cam_1_map','agreement_point')
        _t = self._tfs['agreement_point_to_cam_2_map']
        self._tf_broadcaster.sendTransform(_t['position'],_t['orientation'],rospy.Time.now(),'agreement_point','cam_2_map')

    def _process_ar_tag_agreement(self):
        cam_1 = copy.deepcopy(self._cams["cam_1"]["ar_poses"])
        cam_1_ids = [cam_1[i].id for i in range(0,len(cam_1))]
        cam_2 = copy.deepcopy(self._cams["cam_2"]["ar_poses"])
        cam_2_ids = [cam_2[i].id for i in range(0,len(cam_2))]

        # search for matches
        while len(cam_1) > 0:
            id = cam_1_ids[0]

            try:
                idx = cam_2_ids.index(id)

                # if found in both, average
                tag_c1 = cam_1.pop(0)
                tag_c2 = cam_2.pop(idx)
            except ValueError:
                # if not in second cam, publish as is
                tag_c1 = cam_1.pop(0)

        # those in second cam, publish as well
        for i in range(0,len(cam_2)):


    def _process_block_agreement(self):
        pass


    def _color_slct_cb(self, request):
        status = True
        for cam in self._cams.keys():
            _s = self._cams[cam]["color_slct_srv"](request.min_hue,request.max_hue).status
            status = _s and status
        return ColorSelectResponse(status=status)

    def _ar3_cb(self, cam, message):
        self._cams[cam]["ar_poses"] = message.markers

    def _bk3_cb(self, cam, message):
        self._cams[cam]["bk_poses"] = message.blocks


if __name__ == "__main__":
    rospy.init_node('two_camera_agreement')

    node = TwoCameraAgreement()
    node.spin()
