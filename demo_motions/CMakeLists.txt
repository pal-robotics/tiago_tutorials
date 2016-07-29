cmake_minimum_required(VERSION 2.8.3)
project(demo_motions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  play_motion_msgs
  roscpp
  rospy
)


catkin_package(
)

add_executable(run_loop_node src/run_loop_node.cpp)
target_link_libraries(run_loop_node ${catkin_LIBRARIES})