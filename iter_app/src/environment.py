#!/usr/bin/env python

'''
Environment Node
Author Curt Henrichs
Date 5-16-19

Provides environment context for ITER runner.

'''


MODE_COLLISION_MOVEIT = 'collision_moveit'
MODE_MARKER = 'marker'


import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

rospy.init_node('environment')

mode = rospy.get_param('~mode',MODE_MARKER)
if mode == MODE_COLLISION_MOVEIT:
    import tools.environment_collision_moveit_interface as task_env_interface
elif mode == MODE_MARKER:
    import tools.environment_marker_interface as task_env_interface
else:
    raise Exception('Invalid environment mode selected')


class Environment:

    def __init__(self):
        self._gen_task_objs_srv = rospy.Service("/environment/generate_task_objects",,self._generate_task_objs)
        self._clear_task_objs_srv = rospy.Service("/environment/clear_task_objects",,self._clear_task_objs)
        self._connect_task_obj_srv = rospy.Service("/environment/connect_task_object",,self._connect_task_obj)
        self._release_task_obj_srv = rospy.Service("/environment/release_task_object",,self._release_task_obj)
        self._get_vision_obj_srv = rospy.Service("/environment/get_vision_object",,self._get_vision_obj)
        self._cal_bot_to_cam_srv = rospy.Service("/environment/calibrate_robot_to_camera",,self._cal_bot_to_cam)
        self._get_state_srv = rospy.Service("/environment/get_state",,self._get_state)

    def _generate_task_objs(self, request):
        # Generates new markers of objects defined by array of objects provided
        pass

    def _clear_task_objs(self, request):
        # Clears set of objects defined by array of string IDs (provided as JSON)
        pass

    def _connect_task_obj(self, request):
        # Connects object to robot
        # Provide a pose which is used for release to calculate the transformation
        # over movement used to plot new object
        pass

    def _release_task_obj(self, request):
        # Disconnects object from robot
        # Provide a pose which is used to calculate the transformation over
        # movement used to plot new object
        pass

    def _get_vision_obj(self, request):
        # Finds a object from vision set that meets the criteria given.
        # Converts to task object with ID.
        # Returns pose of object with ID.
        pass

    def _cal_bot_to_cam(self, request):
        # probe camera to robot transform, note robot's ar tag must be within
        # camera's field of view
        pass

    def _get_state(self, request):
        # Parameters include:
        #   - id grasped objects
        #   - id all objects
        #   - vision objects
        #   - ar tags
        pass

if __name__ == "__main__":
    pass
