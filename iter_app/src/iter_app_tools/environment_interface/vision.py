
import rospy

from iter_vision.msg import BlockPose3D, BlockPose3DArray
from iter_vision.srv import ColorSelect, ColorSelectRequest, ColorSelectResponse
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkerArray


BLOCK_UNKNOWN = BlockPose3D.UNKNOWN
BLOCK_SMALL = BlockPose3D.SMALL
BLOCK_LARGE = BlockPose3D.LARGE


ar_tags = {}
blocks = {}


def get_vision_ids():
    global blocks
    return blocks.keys()

def get_ar_ids():
    global ar_tags
    return ar_tags.keys()

def get_block(type):
    global blocks

    if type != None: # attempt block match
        for bid in blocks.keys():
            if blocks[bid][1] == type:
                return bid
    elif len(blocks.keys()) > 0: # get first block found
        return blocks.keys()[0]

    return None # No block with type found

def get_arg_tag(id):
    return ar_tags[id] if id in ar_tags else None

def _ar3_cb(message):
    global ar_tags

    ar_tags = {}
    for m in message.markers:
        ar_tags[str(m.id)] = m.pose.pose

def _bk3_cb(message):
    global blocks

    blocks = {}
    for b in message.blocks:
        blocks[str(b.id)] = (b.pose,b.type)


ar3_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkerArray, _ar3_cb, queue_size=5)
pose_pub = rospy.Subscriber("/block_pose/pose_3d", BlockPose3DArray, _bk3_cb, queue_size=5)

color_slct_srv = rospy.ServiceProxy("/block_vision/color_select",ColorSelect)
def set_vision_params(params):

    if 'default' in params and params['default']:
        min_hue = ColorSelectRequest.MIN_HUE_VALUE
        max_hue = ColorSelectRequest.MAX_HUE_VALUE
    else:
        min_hue = params['min_hue']
        max_hue = params['max_hue']

    return color_slct_srv(min_hue,max_hue).status
