cmake_minimum_required(VERSION 2.8.3)
project(tiago_joint_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(TiagoJointController src/TiagoJointController.cpp)
target_link_libraries(TiagoJointController ${catkin_LIBRARIES})

### INSTALL ###

install(TARGETS TiagoJointController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
