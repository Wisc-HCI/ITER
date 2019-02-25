cmake_minimum_required(VERSION 2.8.3)
project(m1n6s_robotiq85_moveit_config)

find_package(catkin REQUIRED actionlib_msgs)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
