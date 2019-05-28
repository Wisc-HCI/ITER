import sys
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)

ARM_MOVE_GROUP = rospy.get_param("arm_move_group")

arm_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

dynamic_environment_ids = []
grasped_list = []


def generate_dynamic_environment(env_data):
    global dynamic_environment_ids

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
    global dynamic_environment_ids, grasped_list

    id_list = ids if ids is not None else dynamic_environment_ids

    for id in grasped_list:
        eef_link = arm_group_commander.get_end_effector_link()
        scene.remove_attached_object(eef_link, name=id)
    grasped_list = []

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

def connect_obj_to_robot(id, *args):
    eef_link = arm_group_commander.get_end_effector_link()
    touch_links = robot.get_link_names()
    scene.attach_box(eef_link, id, touch_links=touch_links)
    rospy.sleep(1)

    grasped_list.append(id)

def disconnect_obj_from_robot(id, *args):
    eef_link = arm_group_commander.get_end_effector_link()
    scene.remove_attached_object(eef_link, name=id)
    rospy.sleep(1)

    grasped_list.remove(id)
