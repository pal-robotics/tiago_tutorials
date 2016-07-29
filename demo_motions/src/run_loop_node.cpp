

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
  ros::init(argc, argv, "run_motion");

  std::cout << "\n" << "==========================================================================================\n";
  std::cout << "To start using the run_alive program, use '$ rosparam set /tiago_alive/enable_motion true'\n";
  std::cout << "==========================================================================================\n";
  std::cout << "To pause using the run_alive program, use '$ rosparam set /tiago_alive/enable_motion false'\n";
  std::cout << "==========================================================================================\n\n";


  std::basic_string<char> goal_array[] = 
  {
  	//GENERAL MOVEMENTS
  	"easily_amused","how_many_fingers_1", "how_many_fingers_2", "typing", "wipes_brow", "sweep", "adhd",
    
    //MODEL STEEL SPECIFIC MOVEMENTS
    "scratch", 

    //MODEL TITANIUM SPECIFIC MOVEMENTS
    "first_move","how_many_fingers_3", "scratch_stomach", "scratch_head", "beckon" 
  };
  
  int goal_array_size = sizeof(goal_array)/sizeof(goal_array[0]); //obtain size of goal_array
  int check_previous_execution = 0; //stores previously executed motion to be checked when creating random number, so that the same motion is not run twice



  ROS_INFO("Starting run_motion application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

  while(ros::ok())
  {  
    bool myparam = false;
    bool success = nh.getParamCached("/tiago_alive/enable_motion", myparam); //if parameter topic does not exist success will return false
    if(success == true)
    {
      if(myparam == true)//if the rosparam exists, and is set to true, run subsequent code
      {
        ROS_INFO("Waiting for Action Server ...");
        client.waitForServer();
        int random_select = rand() % goal_array_size; //obtain random integer for motion selection

        if(random_select == check_previous_execution) //check against previously executed motion
          random_select = rand() % goal_array_size; //if the same, obtain new random number
        else
        {
          play_motion_msgs::PlayMotionGoal goal;

          goal.motion_name = goal_array[random_select];
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
          
          check_previous_execution = random_select; //set to motion just executed
        }
      }
        else if(myparam==false)//if the enable_motion rosparam false then sleep untill the param is set to true again.
        {
          ros::Rate r(50);
          r.sleep();
        }
      }
      else if(success == false) //if enable_motion rosparam does not exist
        {
          ROS_INFO("Parameter /tiago_alive/enable_motion does not exist");
          return EXIT_FAILURE;
        }
    }
  }
