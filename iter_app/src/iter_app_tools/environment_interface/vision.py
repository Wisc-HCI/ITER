
import rospy


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
                return bid, blocks[bid][0]
    elif len(blocks.keys()) > 0: # get first block found
        bid = blocks.keys()[0]
        return bid, blocks[bid][0]

    return None, None # No block with type found

def get_ar_tag(id):
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

def set_vision_params(params):

    if 'default' in params and params['default']:
        min_hue = ColorSelectRequest.MIN_HUE_VALUE
        max_hue = ColorSelectRequest.MAX_HUE_VALUE
    else:
        min_hue = params['min_hue']
        max_hue = params['max_hue']

    return color_slct_srv(min_hue,max_hue).status
