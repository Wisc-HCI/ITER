import sys
import uuid
import moveit_commander

from iter_app.msg import EnvironmentObject

moveit_commander.roscpp_initialize(sys.argv)

ARM_MOVE_GROUP = rospy.get_param("arm_move_group")

arm_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

dynamic_environment_ids = []
grasped_list = []


def generate_dynamic_environment(env_data):
    global dynamic_environment_ids
    status = True

    for obj in env_data:

        # Determine ID
        id = obj.id
        if id == "":
            id = str(uuid.uuid1().hex)

        # Handle representations
        #   Currently only box
        if obj.representation != EnvironmentObject.REPRESENTATION_BOX:
            status = False
            continue

        dynamic_environment_ids.append(id)

        scene.remove_world_object(id)
        scene.add_box(
            name=id,
            pose=PoseStamped(
                header=Header(frame_id=robot.get_planning_frame()),
                pose=obj.pose),
            size=(obj.size.x, obj.size.y, obj.size.z))

    rospy.sleep(1)
    rospy.loginfo(scene.get_known_object_names())
    print dynamic_environment_ids
    return status

def clear_dynamic_environment(ids=[], all=False):
    global dynamic_environment_ids, grasped_list
    status = True

    # prepare delete list
    if all:
        delete_list = dynamic_environment_ids
        for id in ids:
            delete_list.remove(id)
    else:
        delete_list = ids

    # Remove from grapsing
    for id in grasped_list:
        eef_link = arm_group_commander.get_end_effector_link()
        scene.remove_attached_object(eef_link, name=id)
    grasped_list = []

    # Remove all objects
    for id in delete_list:
        scene.remove_world_object(id)
        try:
            dynamic_environment_ids.remove(id)
        except:
            status = False
            continue

    rospy.sleep(1)
    rospy.loginfo(scene.get_known_object_names())

    return status

def connect_obj_to_robot(id, *args):
    eef_link = arm_group_commander.get_end_effector_link()
    touch_links = robot.get_link_names()
    scene.attach_box(eef_link, id, touch_links=touch_links)
    rospy.sleep(1)

    grasped_list.append(id)
    return True

def disconnect_obj_from_robot(id, *args):
    eef_link = arm_group_commander.get_end_effector_link()
    scene.remove_attached_object(eef_link, name=id)
    rospy.sleep(1)

    grasped_list.remove(id)
    return False

def get_all_task_ids():
    return dynamic_environment_ids

def get_grasped_ids():
    return grasped_list
