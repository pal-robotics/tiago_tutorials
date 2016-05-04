cmake_minimum_required(VERSION 2.8.3)
project(run_motion)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  play_motion_msgs
  roscpp
)


catkin_package(
)

add_executable(run_motion src/run_motion_node.cpp)
target_link_libraries(run_motion ${catkin_LIBRARIES})


## Install

#install(PROGRAMS
#        scripts/run_motion.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS run_motion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
