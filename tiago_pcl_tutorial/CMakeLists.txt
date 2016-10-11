cmake_minimum_required(VERSION 2.8.3)
project(tiago_pcl_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  tf
  sensor_msgs
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
            src/pcl_filters.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(segment_table src/segment_table.cpp)
target_link_libraries(segment_table ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} segment_table
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


