cmake_minimum_required(VERSION 2.8.3)
project(say_something)

find_package(catkin REQUIRED COMPONENTS
  pal_interaction_msgs
  rospy
)

catkin_package()


install(PROGRAMS
  scripts/say.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
