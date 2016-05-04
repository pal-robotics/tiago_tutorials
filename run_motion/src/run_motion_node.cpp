/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */


// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// C++ standard headers
#include <cstdlib>


int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_motion");

  if ( argc < 2 )
  {
      ROS_INFO(" ");
      ROS_INFO("Usage:");
      ROS_INFO(" ");
      ROS_INFO("\trosrun run_motion run_motion MOTION_NAME");
      ROS_INFO(" ");
      ROS_INFO("\twhere MOTION_NAME must be one of the motions listed in: ");
      ROS_INFO_STREAM(std::system("rosparam list /play_motion/motions | grep joints | cut -d'/' -f4"));
      ROS_INFO(" ");
      return EXIT_FAILURE;
  }

  ROS_INFO("Starting run_motion application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

  ROS_INFO("Waiting for Action Server ...");
  client.waitForServer();

  play_motion_msgs::PlayMotionGoal goal;

  goal.motion_name = argv[1];
  goal.skip_planning = false;
  goal.priority = 0;

  ROS_INFO_STREAM("Sending goal with motion: " << argv[1]);
  client.sendGoal(goal);

  ROS_INFO("Waiting for result ...");
  bool actionOk = client.waitForResult(ros::Duration(30.0));

  actionlib::SimpleClientGoalState state = client.getState();

  if ( actionOk )
  {
      ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
  }
  else
  {
      ROS_ERROR_STREAM("Action failed with state: " << state.toString());
  }

  return EXIT_SUCCESS;
}
