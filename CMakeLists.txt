cmake_minimum_required(VERSION 3.0.0)
project(ros_f9p_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

###################
## Catkin Config ##
###################

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/gps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
