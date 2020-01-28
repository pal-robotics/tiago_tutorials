cmake_minimum_required(VERSION 2.8.3)
project(look_to_point)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  cv_bridge
  image_transport
  sensor_msgs
  control_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED 
    COMPONENTS
    opencv_highgui)

catkin_package()

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(look_to_point src/look_to_point.cpp)
target_link_libraries(look_to_point
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

### INSTALL ###

install(TARGETS look_to_point
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
