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


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        scripts/run_motion.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS run_motion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
