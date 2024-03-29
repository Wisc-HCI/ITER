# Interdependence Task Experiment Runner

## Archival Notice
This project has been moved to archive status given ROS 1 nearing EOL and
no active plan to update to ROS 2. Code & artifacts provided "as is" for
reference.

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
The goal is to observe how the different levels of interdependence affect participant's
perceptions of the task as they collaborate with the robot.

## Publications
[Task Interdpendence in Human-Robot Teaming](https://ieeexplore.ieee.org/document/9223555)

> Human-robot teaming is becoming increasingly common within manufacturing processes. A key aspect practitioners need to decide on when developing effective processes is the level of task interdependence between human and robot team members. Task interdependence refers to the extent to which one’s behavior affects the performance of others in a team. In this work, we examine the effects of three levels of task interdependence—pooled, sequential, reciprocalin human-robot teaming on human worker’s mental states, task performance, and perceptions of the robot. Participants worked with the robot in an assembly task while their heart rate variability was being recorded. Results suggested human workers in the reciprocal interdependence level experienced less stress and perceived the robot more as a collaborator than other two levels. Task interdependence did not affect perceived safety. Our ﬁndings highlight the importance of considering task structure in human-robot teaming and inform future research on and industry practices for human-robot task allocation.

```bibtex
@inproceedings{zhao2020task,
  title={Task Interdependence in Human-Robot Teaming},
  author={Zhao, Fangyun and Henrichs, Curt and Mutlu, Bilge},
  booktitle={2020 29th IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={1143--1149},
  year={2020},
  organization={IEEE}
}
```

[Designing Interface Aids to Assist Collaborative Robot Operators in Attention Management](https://ieeexplore.ieee.org/document/9515519)

> As collaborative robots become increasingly widespread in manufacturing settings, there is a greater need for tools and interfaces to support operators who integrate, supervise, and troubleshoot these systems. In this paper, we present an application of the Robot Attention Demand (RAD) metric for use in the design of user interfaces to support operators in collaborative manufacturing scenarios. Building on prior work that introduced RAD, we designed and implemented prototype timeline and countdown-timer interfaces to be used within a collaborative assembly-inspection task where an operator is also responsible for a separate sorting task. We performed a user evaluation to investigate the effects of displaying predictive RAD information on operator performance and perceptions of the task. Our results show lower perceived task load and increased usability scores compared to baseline condition without an interface. These findings suggest that predictive RAD should be used by designers and engineers developing operator interfaces for collaborative robot applications in manufacturing.

```bibtex
@inproceedings{henrichs2021designing,
  title={Designing Interface Aids to Assist Collaborative Robot Operators in Attention Management},
  author={Henrichs, Curt and Zhao, Fangyun and Mutlu, Bilge},
  booktitle={2021 30th IEEE International Conference on Robot \& Human Interactive Communication (RO-MAN)},
  pages={264--271},
  year={2021},
  organization={IEEE}
}
```

## Packages
The following packages are provided:
- [iter](./README.md)
  - Meta-package for ITER system
- [iter_app](./iter_app/README.md)
  - Provides access to ITER application and core launch files
- [iter_tasks](./iter_tasks/README.md)
  - Provides task generation tools and interfacing from plan to runner
- [rad_ui](./rad_ui/README.md)
  - Web interface to present signals from ITER

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

## Requirements:
ITER packages must be installed in your catkin workspace. ITER was developed under ROS Kinetic.

Required ROS Packages:
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - Installing through package manager functioned incorrectly for me using Kinetic on Ubuntu 16.04
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
    - First try the package manager `sudo apt install ros-kinetic-rosauth`
- [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations)
  - Install for Universal Robotics or Kinova as needed.
    - [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
    - [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros)
    - [dniewinski/ur_modern_driver](https://github.com/dniewinski/ur_modern_driver)
    - [industrial_core](https://wiki.ros.org/industrial_core) via `apt install ros-<VERSION>-industrial-core`
    - [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
- [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
  - Note if using Relaxed-IK, then you may ignore MoveIt setup
  - [Wisc-HCI/robot_behavior](https://github.com/Wisc-HCI/robot_behavior)
- [MoveIt](moveit.ros.org)
  - `sudo apt-get install ros-kinetic-moveit`
- [Wisc-HCI/ros_pop_button](https://github.com/Wisc-HCI/ros_pop_button)
- [timed_roslaunch](http://wiki.ros.org/timed_roslaunch)


Other Requirements:
- (For RAD UI) install npm and run `npm install` inside `rad_ui/src/websites` subdirectory
  - Note cannot use `nodejs`, scripts expect the name to be `node`!

Experimental / Optional Packages:
- [Wisc-HCI/magnetic_block_detection](https://github.com/Wisc-HCI/magnetic_block_detection)

## Run
Enter following into terminal to run ITER application in simulation

```
roslaunch iter_app main.launch robot:=ur3e simulated:=true planner:=rik
```

Enter following into terminal to run ITER application on robot

```
roslaunch iter_app main.launch robot:=ur3e simulated:=false planner:=ur
```
