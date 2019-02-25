# Experiment Therblig Task Runner

##  Overview
Task runner ROS will run set of therbligs on UR3/UR5 robotic arms. Developed to
run experiments in task dependence and RAD signaling. The following nodes were
developed.

* Therblig_Runner
  * ROS node to manage robot behavior from therbligs and button input.
  * Computes RAD signal, neglect time, and interaction time.
  * Command line interface to trigger experimental conditions.
  * [Documentation](/Experiments.md)
* RAD_UI
  * Presents RAD signal to user. Ignore if not being used for study.
  * [Documentation](/rad_ui/README.md)
* Button
  * Interfaces with smart button for participant input while interacting with robot.

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

### Requirements:
- Must be installed as a ROS package in a ROS Workspace
- Other required packages:
  - [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
    - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
  - [ros-planning/moveit_robots](https://github.com/ros-planning/moveit_robots)
  - [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
  - [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros)
  - [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)

## Run
Enter following into terminal,

'''
roslaunch iter_app iter.launch robot:=ur5 simulated:=true
'''


Note: try checking the frames and joints for name diff
ROS bags trajectory replay
Pickle 