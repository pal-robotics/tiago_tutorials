// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "control_msgs/action/point_head.hpp"

constexpr char CAMERA_FRAME[] = "head_front_camera_rgb_optical_frame";
constexpr char ARUCO_TOPIC[] = "aruco_single/pose";

using namespace std::placeholders;

class LookToAruco : public rclcpp::Node
{
private:
  // Intrinsic parameters of the camera
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
  rclcpp_action::Client<control_msgs::action::PointHead>::SharedPtr action_client_ptr_;

public:
  LookToAruco(/* args */)
  : rclcpp::Node("look_to_aruco")
  {
    this->action_client_ptr_ = rclcpp_action::create_client<control_msgs::action::PointHead>(
      this,
      "/head_controller/point_head_action");

    int iterations = 0, max_iterations = 3;
    if (!this->action_client_ptr_->wait_for_action_server(std::chrono::milliseconds(2000)) &&
      rclcpp::ok() && iterations < max_iterations)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for the point_head_action server to come up");
      ++iterations;
    }
    if (iterations == max_iterations) {
      RCLCPP_ERROR(
        this->get_logger(), "Error: head controller action server not available!. Aborting");
      rclcpp::shutdown();
      throw std::runtime_error("Error: head controller action server not available!. Aborting");
    }
  }

  ~LookToAruco()
  {
  }

  bool setup()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to " << ARUCO_TOPIC << "...");
    aruco_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      ARUCO_TOPIC, 10,
      std::bind(&LookToAruco::aruco_callback, this, _1));

    return true;
  }

  // ROS callback when detecting aruco marker to look at the Aruco
  void aruco_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Aruco detected: looking to it");
    geometry_msgs::msg::PointStamped point_stamped;

    point_stamped.header.frame_id = msg->header.frame_id;
    point_stamped.header.stamp = msg->header.stamp;

    point_stamped.point.x = msg->pose.position.x;
    point_stamped.point.y = msg->pose.position.y;
    point_stamped.point.z = msg->pose.position.z;

    // build the action goal
    control_msgs::action::PointHead::Goal goal;
    // the goal consists in making the Z axis of the CAMERA_FRAME to point towards the point_stamped
    goal.pointing_frame = CAMERA_FRAME;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.min_duration.sec = 1.0;
    goal.max_velocity = 1.0;
    goal.target = point_stamped;

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "point_stamped.point.x " << point_stamped.point.x
      << " point_stamped.point.y " << point_stamped.point.y
      << " point_stamped.point.z" << point_stamped.point.z);
    this->action_client_ptr_->async_send_goal(goal);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LookToAruco> look_to_aruco = std::make_shared<LookToAruco>();
  if (look_to_aruco->setup()) {
    rclcpp::spin(look_to_aruco);
  }
}
