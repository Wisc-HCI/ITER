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

## Notes
Note: try checking the frames and joints for name diff
ROS bags trajectory replay
Pickle



## Ref for later
https://github.com/JamesGiller/ifttt-ros
Note to self, https://ieeexplore.ieee.org/document/5641726
