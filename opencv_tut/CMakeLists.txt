cmake_minimum_required(VERSION 2.8.3)
project(opencv_tut)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  pal_detection_msgs
  cv_bridge
  geometry_msgs
  message_generation
)

find_package(OpenCV 2.4 REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_message_files(
  FILES
  valueMatrix.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 )

catkin_package(
  LIBRARIES opencv_tut
  CATKIN_DEPENDS roscpp rospy image_transport cv_bridge pal_detection_msgs  geometry_msgs message_generation
  DEPENDS system_lib
)


include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(track_sequential src/track_sequential.cpp)
  target_link_libraries(track_sequential yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(track_sequential ${PROJECT_NAME}_generate_messages_cpp)

add_executable(corner_detection src/corner_detection.cpp)
  target_link_libraries(corner_detection yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(corner_detection ${PROJECT_NAME}_generate_messages_cpp)

add_executable(find_keypoints src/find_keypoints.cpp)
  target_link_libraries(find_keypoints yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(find_keypoints ${PROJECT_NAME}_generate_messages_cpp)

add_executable(flann_matching src/flann_matching.cpp)
  target_link_libraries(flann_matching yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(flann_matching ${PROJECT_NAME}_generate_messages_cpp)