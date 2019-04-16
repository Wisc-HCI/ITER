import time
import rospy
import moveit_commander

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

GRIPPER_MOVE_GROUP = rospy.get_param("gripper_move_group")

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
dynamic_environment_ids = []

def generate_dynamic_environment(env_data):
    for obj in env_data:

        if obj['representation'] != 'box':
            raise('Invalide object representation')

        dynamic_environment_ids.append(obj['name'])

        scene.remove_world_object(obj['name'])
        scene.add_box(
            name=obj['name'],
            pose=PoseStamped(
                header=Header(frame_id=robot.get_planning_frame()),
                pose=Pose(
                    position=Point(
                        x=obj['position']['x'],
                        y=obj['position']['y'],
                        z=obj['position']['z']),
                    orientation=Quaternion(
                        x=obj['orientation']['x'],
                        y=obj['orientation']['y'],
                        z=obj['orientation']['z'],
                        w=obj['orientation']['w']))),
            size=(obj['size']['x'], obj['size']['y'], obj['size']['z']))

    rospy.sleep(1)
    rospy.loginfo(scene.get_known_object_names())
    print dynamic_environment_ids


def clear_dynamic_environment(ids=None):
    id_list = ids if ids is not None else dynamic_environment_ids

    remove_list = []
    for id in id_list:

        scene.remove_world_object(id)
        remove_list.append(id)

    for id in remove_list:
        try:
            dynamic_environment_ids.remove(id)
        except ValueError:
            pass

    rospy.sleep(1)
    rospy.loginfo(scene.get_known_object_names())
