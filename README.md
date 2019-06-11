# Interdependence Task Experiment Runner

##  Overview
Interdependence Task Experiment Runner (ITER) implements two tasks regarding
human-robot interdependence in a collaborative manufacturing setting. Both of
these tasks make use of a single collaborative robotic arm to perform simplified
tasks such as constructing a block house.

Task one is based on robot attention demand (RAD) which is defined by a ratio
of interaction time relative to total task time. Total time can be broken into
interaction time (where the human interacts with the robot) and neglect time
(where the robot is able to work without human intervention/interaction). Of
interest in this, is the research question of presenting this neglect time to
the human collaborator in order to decrease wasted time where the robot must wait
for the human. This will be measured by lowered interaction time if the RQ's
hypothesis is found.

Task two is based on levels of interdependence in human robot collaboration.
Specifically, looking to pooled, sequential, and reciprocal interdependence.
...TODO finish this description...

## Packages
The following packages are provided:
- [iter_app](./iter_app/README.md)
  - Provides access to ITER application and core launch files
- [iter_vision](./iter_vision/README.md)
  - Provides vision subsystem for finding blocks defined by AR boundary
- [ar_track_alvar](./ar_track_alvar/README.md)
  - modified open-source alvar AR tag tracker to also report 2D pose of tags

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

### Requirements:
Must be installed as a ROS package in a catkin workspace

Required packages:
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)
  - Install for UR or Kinova as needed
- (optional) [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
  - See iter_app src/configs for pre-written setup files
  - Note if using Relaxed-IK, then ignore MoveIt and robot_configurations
  - [Wisc-HCI/robot_behavior](https://github.com/Wisc-HCI/robot_behavior)
- usb-cam
  - `sudo apt-get install ros-kinetic-usb-cam`
- MoveIt
  - `sudo apt-get install ros-kinetic-moveit`

Other Requirements:
- install opencv and python wrapper (instructions for Ubuntu/Debian)
  - `sudo apt-get install libopencv-dev python-opencv`
- (for RAD UI) install npm and run `npm install` inside iter_app/rad_ui directory

## Run
Enter following into terminal to run ITER application

```
roslaunch iter_app main.launch robot:=ur5 simulated:=true
```

## Notes / Todo
- https://github.com/Wisc-HCI/hci_demos/blob/master/mimicry_control/bin/mico_controller.py
