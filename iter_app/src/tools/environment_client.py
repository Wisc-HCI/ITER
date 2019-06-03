import rospy

from iter_app.srv import GetVisionObject, GetVisionObjectResponse
from iter_app.srv import ClearTaskObjects, ClearTaskObjectsResponse
from iter_app.srv import ConnectTaskObject, ConnectTaskObjectResponse
from iter_app.srv import ReleaseTaskObject, ReleaseTaskObjectResponse
from iter_app.srv import GenerateTaskObjects, GenerateTaskObjectsResponse
from iter_app.srv import GetEnvironmentState, GetEnvironmentStateResponse
from iter_app.srv import CalibrateRobotToCamera, CalibrateRobotToCameraResponse

class EnvironmentClient:

    def __init__(self):
        self.generate_task_objects = rospy.ServiceProxy("/environment/generate_task_objects",GenerateTaskObjects)
        self.clear_task_objects = rospy.ServiceProxy("/environment/clear_task_objects",ClearTaskObjects)
        self.connect_task_object = rospy.ServiceProxy("/environment/connect_task_object",ConnectTaskObject)
        self.release_task_object = rospy.ServiceProxy("/environment/release_task_object",ReleaseTaskObject)
        self.get_vision_object = rospy.ServiceProxy("/environment/get_vision_object",GetVisionObject)
        self.calibrate_robot_to_camera = rospy.ServiceProxy("/environment/calibrate_robot_to_camera",CalibrateRobotToCamera)
        self.get_state = rospy.ServiceProxy("/environment/get_state",GetEnvironmentState)
