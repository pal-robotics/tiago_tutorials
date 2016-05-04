cmake_minimum_required(VERSION 2.8.3)
project(look_hand)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  rospy
  tf
)

catkin_package(
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/look_hand.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
