cmake_minimum_required(VERSION 2.8.3)
project(demo_motions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  play_motion_msgs
  roscpp
  rospy
  roslib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


catkin_package(
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(run_loop_node src/run_loop_node.cpp)
target_link_libraries(run_loop_node yaml-cpp ${catkin_LIBRARIES})

add_dependencies(run_loop_node ${catkin_EXPORTED_TARGETS})
