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

add_executable(run_motion src/run_motion_node.cpp)
target_link_libraries(run_motion ${catkin_LIBRARIES})

add_executable(run_loop_node src/run_loop_node.cpp)
target_link_libraries(run_loop_node ${catkin_LIBRARIES})

## Install

install(PROGRAMS
        scripts/run_motion_python_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS run_motion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
