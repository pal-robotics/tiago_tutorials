#ifndef TIAGOBASECONTROLLER_H
#define TIAGOBASECONTROLLER_H

/**
 *  TiagoBaseController for TIAGo robot. Control Tiago's base forward/backward, and turn right/left.
 *  
 *  @author derzu
 *
 *  Compile isolated this file :
 *  g++ TiagoBaseController.cpp -o TiagoBaseController -I/opt/ros/indigo/include -I../include/ -I. -Iinclude/ -L/opt/ros/indigo/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib
 *
 *  To run this controller with ROSRun:
 *  $ rosrun tiago_base_controller TiagoBaseController
 *
 *  Inspired on:
 *  http://wiki.ros.org/p2os-purdue/Tutorials/C%2B%2B%20Velocity%20Controller%20for%20P2OS%20Robots
 *  http://answers.ros.org/question/65406/p2os-groovy-c-velocity-and-c-pose-listener/
 **/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TiagoBaseController {
	public:
		// Controllers Constants
		static const float NONE     ;//=  0.0f;
		static const float LEFT     ;//=  0.5f;
		static const float RIGHT    ;//= -0.5f;
		static const float FORWARD  ;//=  0.5f;
		static const float BACKWARD ;//= -0.5f;
		
		TiagoBaseController();
		virtual ~TiagoBaseController();
		void createClients();

		void executeGoal(float direction, float angle);
		void executeGoal(geometry_msgs::Twist cmd_msg);

	private:
		void init();

		ros::NodeHandle *n;

		ros::Publisher cmd_pub;
		
		geometry_msgs::Twist cmd_msg;
};

#endif

