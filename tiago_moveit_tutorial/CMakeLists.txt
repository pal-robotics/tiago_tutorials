cmake_minimum_required(VERSION 2.8.3)
project(tiago_moveit_tutorial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  geometry_msgs
)

catkin_package(
)


include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(plan_arm_torso_fk src/plan_arm_torso_fk.cpp)
add_dependencies(plan_arm_torso_fk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_arm_torso_fk
   ${catkin_LIBRARIES}
 )

add_executable(plan_arm_torso_ik src/plan_arm_torso_ik.cpp)
add_dependencies(plan_arm_torso_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_arm_torso_ik
   ${catkin_LIBRARIES}
 )

install(TARGETS plan_arm_torso_fk plan_arm_torso_ik
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
