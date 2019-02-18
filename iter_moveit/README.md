## Kinova

'''
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s300

roslaunch m1n6s300_moveit_config m1n6s300_demo.launch
'''

## UR 5

'''
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=10.130.229.132 [reverse_port:=REVERSE_PORT]

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch

roslaunch ur5_moveit_config moveit_rviz.launch config:=true
'''
