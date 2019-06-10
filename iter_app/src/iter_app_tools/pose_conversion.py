'''

'''

from geometry_msgs.msg import Pose, Vector3, Quaternion


def pose_dct_to_msg(dct):
    msg = Pose(position=Vector3(x=dct['position']['x'],
                                y=dct['position']['y'],
                                z=dct['position']['z']),
              orientation=Quaternion(x=dct['orientation']['x'],
                                     y=dct['orientation']['y'],
                                     z=dct['orientation']['z'],
                                     w=dct['orientation']['w']))
    return msg

def pose_msg_to_dct(msg):
    dct = {
            "position": {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            }
        }
    return dct

def pose_msg_to_tf(msg):
    pos = (msg.position.x,msg.position.y,msg.position.z)
    rot = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    return pos,rot

def pose_dct_to_tf(dct):
    pos = (dct['position']['x'],dct['position']['y'],dct['position']['z'])
    rot = (dct['orientation']['x'],dct['orientation']['y'],dct['orientation']['z'],dct['orientation']['w'])
    return pos, rot
