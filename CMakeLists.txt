cmake_minimum_required(VERSION 2.8.3)
project(tiago_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(look_to_point src/look_to_point.cpp)
target_link_libraries(look_to_point
  ${catkin_LIBRARIES})

### INSTALL ###

install(TARGETS look_to_point
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
   scripts/look_hand.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
