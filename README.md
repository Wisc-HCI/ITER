# Experiment Therblig Task Runner

##  Overview
Task runner ROS will run set of therbligs on UR3/UR5 robotic arms. Developed to
run experiments in task dependence and RAD signaling. The following subpackages
are provided.

* iter_app
  * Provides access to ITER application and core launch files
* iter_moveit
  * m1n6s_robotiq85_moveit_config
    * Custom robot configuration for Kinova Mico and Robotiq control in Moveit
  * m1n6s200_moveit_config
    * Kinova Mico 2 finger gripper which is not provided in kinova package

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

### Requirements:
- Must be installed as a ROS package in a ROS Workspace
- Other required packages:
  - [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
    - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
  - [ros-planning/moveit_robots](https://github.com/ros-planning/moveit_robots)
  - [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
    - Must be in `hic-kinova-master` branch
  - [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros)
  - [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)
    - [ros-industrial/ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)
  - [uos/rospy_message_converter](https://github.com/uos/rospy_message_converter.git)

## Run
Enter following into terminal to run ITTER application

'''
roslaunch iter_app iter.launch robot:=ur5 simulated:=true
'''

## Robots
  - ur3
  - ur5
  - mico-2
  - mico-3
  - mico-robotiq85
