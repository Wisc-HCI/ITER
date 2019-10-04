'''

'''

import rospy

from iter_app.srv import GetARTagPose
from iter_app.srv import SetVisionParams
from iter_app.srv import GetVisionObject
from iter_app.srv import ClearTaskObjects
from iter_app.srv import ConnectTaskObject
from iter_app.srv import ReleaseTaskObject
from iter_app.srv import GenerateTaskObjects
from iter_app.srv import GetEnvironmentState
from iter_app.srv import CalibrateRobotToCamera


class EnvironmentClient:

    def __init__(self):
        self.generate_task_objects = rospy.ServiceProxy("/environment/generate_task_objects",GenerateTaskObjects)
        self.clear_task_objects = rospy.ServiceProxy("/environment/clear_task_objects",ClearTaskObjects)
        self.connect_task_object = rospy.ServiceProxy("/environment/connect_task_object",ConnectTaskObject)
        self.release_task_object = rospy.ServiceProxy("/environment/release_task_object",ReleaseTaskObject)
        self.get_vision_object = rospy.ServiceProxy("/environment/get_vision_object",GetVisionObject)
        self.calibrate_robot_to_camera = rospy.ServiceProxy("/environment/calibrate_robot_to_camera",CalibrateRobotToCamera)
        self.get_state = rospy.ServiceProxy("/environment/get_state",GetEnvironmentState)
        self.set_vision_params = rospy.ServiceProxy("/environment/set_vision_params",SetVisionParams)
        self.get_ar_tag_pose = rospy.ServiceProxy("/environment/get_ar_tag_pose",GetARTagPose)
