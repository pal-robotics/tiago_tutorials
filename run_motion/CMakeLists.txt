cmake_minimum_required(VERSION 2.8.3)
project(run_motion)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  play_motion_msgs
  roscpp
  rospy
)


catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(run_motion src/run_motion_node.cpp)
target_link_libraries(run_motion ${catkin_LIBRARIES})

add_dependencies(run_motion ${catkin_EXPORTED_TARGETS})

## Install

install(PROGRAMS
        scripts/run_motion_python_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS run_motion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
