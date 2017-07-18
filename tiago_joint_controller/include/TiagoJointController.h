#ifndef TIAGOJOINTCONTROLLER_H
#define TIAGOJOINTCONTROLLER_H

/**
 *  TiagoJointController for TIAGo robot.
 *  
 *  @author derzu
 *
 *  Compile isolated this file :
 *  g++ TiagoJointController.cpp -o TiagoJointController -I/opt/ros/indigo/include -I../include/ -L/opt/ros/indigo/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib
 *
 *  If you dont know the joints and the possible values test with the rqt_joint_trajectory_controller:
 *  $ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
 * 
 *  To run this controller with ROSRun:
 *  $ rosrun tiago_joint_controller TiagoJointController
 **/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// A typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> control_client;
typedef boost::shared_ptr<control_client>  control_client_Ptr;

#define TOT 4

class TiagoJointController {
	public:
		// Controllers Constants
		static const int ARM   = 0;
		static const int TORSO = 1;
		static const int HEAD  = 2;
		static const int HAND  = 3;
		
		TiagoJointController();
		TiagoJointController(bool gripper);
		virtual ~TiagoJointController();

		void setGoal(const char * joint, float value);
		void execute(bool sendAll);
		void execute();

	private:
		void init();
		void initGoal(control_msgs::FollowJointTrajectoryGoal &goal);
		control_msgs::FollowJointTrajectoryGoal initArmGoal();
		control_msgs::FollowJointTrajectoryGoal initHeadGoal();
		control_msgs::FollowJointTrajectoryGoal initTorsoGoal();
		control_msgs::FollowJointTrajectoryGoal initHandGoal();
		control_msgs::FollowJointTrajectoryGoal initGripperGoal();
		void createClients();
				
		// Index of the last controller setted
		int lastController;
		
		// If the robot hand is a hand or a gripper. true if gripper (steel), false if normal hand (titanium).
		bool gripper;
		
		// Send all the controlers to be executed at one time just one (the lastController used)
		bool sendAll; // default is false

		// Vector of the clients and goals for the TIAGo
		control_client_Ptr clients[TOT];
		control_msgs::FollowJointTrajectoryGoal goals[TOT];

};

#endif

