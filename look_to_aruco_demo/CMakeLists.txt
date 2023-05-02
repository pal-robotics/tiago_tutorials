cmake_minimum_required(VERSION 3.5)
project(look_to_aruco_demo)

find_package(ament_cmake_auto REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)
# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(look_to_aruco src/look_to_aruco.cpp)
target_include_directories(look_to_aruco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(look_to_aruco PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  look_to_aruco
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "cv_bridge"
  "geometry_msgs"
  "image_transport"
  "sensor_msgs"
)

install(TARGETS look_to_aruco
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
