#!/usr/bin/env python
import ipdb
import sys
import rospy
import moveit_commander as mc
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

if __name__ == '__main__':
    mc.roscpp_initialize(sys.argv)
    rospy.init_node("ur5_moveit_test", anonymous=True)
    robot = mc.RobotCommander()
    psi = mc.PlanningSceneInterface()

    rospy.sleep(1)

    psi.remove_world_object("my_sphere")
    psi.add_sphere(
        name="my_sphere",
        pose=PoseStamped(
            header=Header(frame_id=robot.get_planning_frame()),
            pose=Pose(position=Point(x=1, y=1 ,z=1), orientation=Quaternion(x=0, y=0, z=0, w=1))),
        radius=0.1)

    psi.remove_world_object("my_box")
    psi.add_box(
        name="my_box",
        pose=PoseStamped(
            header=Header(frame_id=robot.get_planning_frame()),
            pose=Pose(position=Point(x=1.2, y=1 ,z=1), orientation=Quaternion(x=0, y=0, z=0, w=1))),
        size=(0.1, 0.1, 0.1))

    psi.remove_world_object("my_plane")
    psi.add_plane(
        name="my_plane",
        pose=PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=robot.get_planning_frame()),
            pose=Pose(position=Point(x=0, y=0 ,z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))),
        normal=(0,0,1),
        offset=1)

    rospy.sleep(1)
    rospy.loginfo(psi.get_known_object_names())
    mc.roscpp_shutdown()
