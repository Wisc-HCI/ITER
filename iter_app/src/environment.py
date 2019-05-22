#!/usr/bin/env python

'''
Environment Node
Author Curt Henrichs
Date 5-16-19

Provides environment context for ITER runner.
- Captures Block pose information from vision system
- Handles Camera to Robot calibrated mapping in tf tree
- Allows task specfic, pre-defined geometery between robot, camera, and tables
'''

import sys
import copy
import time
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

rospy.init_node('environment')

#===============================================================================
#   Runner Environment Representation
#===============================================================================

dynamic_environment_ids = []
grasped_list = []

USE_MOVEIT = rospy.get_param('use_collision',False)
if USE_MOVEIT:
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    ARM_MOVE_GROUP = rospy.get_param("arm_move_group")

    arm_group_commander = moveit_commander.MoveGroupCommander(ARM_MOVE_GROUP)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

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

    def connect_obj_to_robot(id):
        eef_link = arm_group_commander.get_end_effector_link()
        touch_links = robot.get_link_names()
        scene.attach_box(eef_link, id, touch_links=touch_links)
        rospy.sleep(1)

        grasped_list.append(id)

    def disconnect_obj_from_robot(id):
        eef_link = arm_group_commander.get_end_effector_link()
        scene.remove_attached_object(eef_link, name=id)
        rospy.sleep(1)

        grasped_list.remove(id)
else:

    #TODO define markers to display environment

    def generate_dynamic_environment(env_data):
        #TODO
        pass

    def clear_dynamic_environment(ids=None):
        #TODO
        pass

    def connect_obj_to_robot(id):
        #TODO
        pass

    def disconnect_obj_from_robot(id):
        #TODO
        pass

#===============================================================================
#   Vision Environment representation
#===============================================================================

#===============================================================================
#   Node
#===============================================================================

class Environment:

    def __init__(self):
        pass

    def _generate_task_objs(self, message):
        # Generates new markers of objects defined by json structure
        # [
        #   {
        #       "representation": "box",
        #       "name": <some name string>
        #       "size": {"x":<number>,"y":<number>,"z":<number>},
        #       "position": {"x":<number>,"y":<number>,"z":<number>},
        #       "orientation": {"x":<number>,"y":<number>,"z":<number>, "w":<number>}
        #   },
        #   ...
        # ]
        pass

    def _clear_task_objs(self, message):
        # Clears set of objects defined by array of string IDs (provided as JSON)
        pass

    def _connect_task_obj(self, message):
        # Connects object to robot
        # Provide a pose which is used for release to calculate the transformation
        # over movement used to plot new object
        pass

    def _release_task_obj(self, message):
        # Disconnects object from robot
        # Provide a pose which is used to calculate the transformation over
        # movement used to plot new object
        pass

    def _get_vision_obj(self, message):
        # Finds a object from vision set that meets the criteria given.
        # Converts to task object with ID.
        # Returns pose of object with ID.
        pass

    def _param_update(self, message):
        # Parameters include:
        #   - camera to robot transform
        pass

    def _param_retrieve(self, message):
        # Parameters include:
        #   - camera location to robot transform
        #   - ar image to world transform
        #   - id grasped objects
        #   - id all objects
        #   - vision objects
        #   - ar tags
        pass

if __name__ == "__main__":
    pass
