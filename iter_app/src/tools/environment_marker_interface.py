import tf
import copy
import uuid
import rospy

from std_msgs.msg import Bool, ColorRGBA
from iter_app.msg import EnvironmentObject
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

def _refresh(*args):
    global marker_pub, markers

    self.marker_pub.publish([markers[id] for id in markers.keys()])

def generate_dynamic_environment(env_data):
    global markers, reference_frame
    status = True

    for obj in env_data:
        # Determine ID
        id = obj.id
        if id == "":
            id = str(uuid.uuid1().hex)

        # Create base marker message
        markers[id] = Marker(id=id,ns='markers')
        markers[id].header.frame_id = reference_frame
        markers[id].color = ColorRGBA(r=0,b=0,g=1,a=1)

        # Handle representations
        #   Currently only box
        if obj.representation == EnvironmentObject.REPRESENTATION_BOX:
            markers[id].type = Marker.CUBE
        else:
            status = False
            del markers[id]
            continue

        # Update marker pose and size
        markers[id].scale = obj.size
        markers[id].pose = obj.pose

    _refresh()
    return status

def clear_dynamic_environment(ids=[], all=False):
    global markers, grasped
    status = True

    # prepare delete list
    if all:
        delete_list = markers.keys()
        for id in ids:
            delete_list.remove(id)
    else:
        delete_list = ids

    # publish delete message to nodes
    for id in delete_list:
        try:
            markers[id].action = Marker.DELETE
        except:
            status = False
            continue
    _refresh()

    # clear markers and grasps
    for id in delete_list:
        try:
            del markers[id]
            if id in grasped.keys():
                del grasped[id]
        except:
            status = False
            continue

    return status

def connect_obj_to_robot(id, pose):
    global grasped

    if id not in grasped.keys():
        grased[id] = pose
    else:
        return False

def disconnect_obj_from_robot(id, pose):
    global grasped

    if id in grasped.keys():
        #TODO update with a new pose
        del grasped[id]
    else:
        return True

def get_all_task_ids():
    global markers

    return markers.keys()

def get_grasped_ids():
    global grasped

    return grasped.keys()


markers = {}
grasped = {}

marker_topic = rospy.get_param('~marker_topic','/environment/markers')
reference_frame = rospy.get_param('~reference_frame','base_link')
marker_pub = rospy.Publisher(marker_topic,MarkerArray,queue_size=5)
refresh_sub = rospy.Subscriber(marker_topic+'/refresh',Bool,_refresh)
