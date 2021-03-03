# Interdependence Task Experiment Runner - Application

##  Overview
Application provides platform to run interdependence experiments on physical
and virtual robots. This system will aggregate robot control, RAD timing, task runner server, button interface when started. Must be used in conjunction with task runner clients from [iter_tasks](../iter_tasks/README.md).

Checkout main [README](../README.md) for full details.

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

## Run
Enter following into terminal to run ITER application

```
roslaunch iter_app main.launch robot:=ur3e simulated:=true planner:=ur
```

### Robots
  - Universal Robots UR3e (ur3e)
  - Universal Robots UR5 (ur5)
  - Kinova Mico + 2 Finger Gripper (mico-2)
  - Kinova Mico + 3 Finger Gripper (mico-3)
  - Kinova Mico + Robotiq85 (mico-robotiq85)

### Planners
  - ur (ur3e and ur5 only)
  - rik
  - moveit
