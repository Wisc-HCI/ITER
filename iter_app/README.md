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
roslaunch iter_app main.launch robot:=ur5 simulated:=true
```

## Robots
  - ur3
  - ur5
  - mico-2
  - mico-3
  - mico-robotiq85
