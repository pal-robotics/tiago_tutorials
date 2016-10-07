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
/** \author Job van Dieten. */

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// C++ standard headers
#include <cstdlib>
#include <yaml-cpp/yaml.h>


int main(int argc, char** argv)
{
  std::string model = ros::getROSArg(argc, argv, "robot");

  ros::init(argc, argv, "run_motion");
  std::cout << "\n" << "==========================================================================================\n";
  std::cout << "To start using the run_alive program, use '$ rosparam set /tiago_alive/enable_motion true'\n";
  std::cout << "==========================================================================================\n";
  std::cout << "To pause using the run_alive program, use '$ rosparam set /tiago_alive/enable_motion false'\n";
  std::cout << "==========================================================================================\n\n";
  
  std::vector<std::string> motions_vec, descriptions_vec;
  std::string yaml_name = ros::package::getPath("demo_motions") + "/resources/custom_motions_" + model + ".yaml";

  YAML::Node node;
  node = YAML::LoadFile(yaml_name);
  YAML::Node node_play_motion = node["play_motion"];
  YAML::Node node_motions = node_play_motion["motions"];

  for(YAML::const_iterator it=node_motions.begin(); it!= node_motions.end(); it++)
  {
    motions_vec.push_back(it->first.as<std::string>());
    YAML::Node node_motion = node_motions[it->first.as<std::string>()];
    YAML::Node node_meta   = node_motion["meta"];
    descriptions_vec.push_back(node_meta["description"].as<std::string>());
  }

  std::cout << "\n" << "These are the motions loaded for the "<< model << "model:" << "\n";

  for(auto k : motions_vec)
    std::cout << " - " << k << "\n";

  int goal_array_size = motions_vec.size();
  int check_previous_execution = 0; 

  ROS_INFO("Starting run_motion application ...");

  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) 
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

  while(ros::ok())
  {  
    bool myparam = false;
    bool success = nh.getParamCached("/tiago_alive/enable_motion", myparam);
    std::vector<int> last_motions_executed;
    if(success == true)
    {
      if(myparam == true)
      {
        ROS_INFO("Waiting for Action Server ...");
        client.waitForServer();
        int random_select = rand() % goal_array_size;

        //if the selected motion has been already executed
        while (std::find(last_motions_executed.begin(), last_motions_executed.end(), random_select) != last_motions_executed.end())
            random_select = rand() % goal_array_size;

        play_motion_msgs::PlayMotionGoal goal;

        goal.motion_name = motions_vec[random_select];
        if ( descriptions_vec[random_select] == "skip planning")
            goal.skip_planning = true;
        else
            goal.skip_planning = false;
        goal.priority = 0;

        ROS_INFO_STREAM("Sending goal with motion: " << goal.motion_name);
        client.sendGoal(goal);

        ROS_INFO("Waiting for result ...");
        bool actionOk = client.waitForResult(ros::Duration(90.0));

        actionlib::SimpleClientGoalState state = client.getState();

        if ( actionOk )
            ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
        else
            ROS_ERROR_STREAM("Action failed with state: " << state.toString());

        //check if all motions have been executed
        if ( last_motions_executed.size() == motions_vec.size() - 1 )
            last_motions_executed.clear();

        last_motions_executed.push_back(random_select);
      }
      else if(myparam==false)
       {
         ros::Rate r(50);
         r.sleep();
       }
    }
     else if(success == false)
    {
      ROS_INFO("Parameter /tiago_alive/enable_motion does not exist");
      return EXIT_FAILURE;
    }
  }
}
