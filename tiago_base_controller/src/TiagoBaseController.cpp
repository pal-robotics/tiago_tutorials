// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

#include "TiagoBaseController.h"

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

// Controllers Constants
const float TiagoBaseController::NONE     =  0.0f;
const float TiagoBaseController::LEFT     =  0.8f;
const float TiagoBaseController::RIGHT    = -0.8f;
const float TiagoBaseController::FORWARD  =  0.8f;
const float TiagoBaseController::BACKWARD = -0.8f;


TiagoBaseController::TiagoBaseController() {
	init();
}

/**
 * Init the ROS node, clients controllers and goals.
 **/
void TiagoBaseController::init() {
	int argc=1;
	char name[] = "TiagoBaseController";
	char * argv = (char *) name;
	
        cmd_msg.linear.x = NONE;
        cmd_msg.linear.y = NONE;
        cmd_msg.linear.z = NONE;
        cmd_msg.angular.x = NONE;
        cmd_msg.angular.y = NONE;
        cmd_msg.angular.z = NONE;
	
	// Init the ROS node
	ros::init(argc, &argv, "TiagoBaseController");

	ROS_INFO("Starting TiagoBaseController");
	
	n = new ros::NodeHandle();

	// Precondition: Valid clock
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
	{
		ROS_FATAL("Timed-out waiting for valid time.");
	}
	
	// Create an controllers clients to move the TIAGo's joints
	createClients();
}

TiagoBaseController::~TiagoBaseController() {
	if (n)
		delete n;
}


/**
 * Create a Publisher control de base.
 **/
void TiagoBaseController::createClients()
{
	ROS_INFO("Creating velocity clients");

	cmd_pub = n->advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 100);
	
	ros::Rate loop_rate(10);
}



/**
 * Create a command with the direction and angle setted and excute the movement.
 **/
void TiagoBaseController::executeGoal(float direction, float angle) {
	cmd_msg.linear.x  = direction;
	cmd_msg.angular.z = angle;
	
        executeGoal(cmd_msg);
}


/**
 * Excute the command with movement directions.
 **/
void TiagoBaseController::executeGoal(geometry_msgs::Twist cmd_msg) {
        cmd_pub.publish(cmd_msg);
        
        ros::spinOnce();
}

using namespace std;

/**
 * Test the TiagoBaseController
 * Rename mainTest to main, to test just this file.
 **/
int main(int argc, char** argv)
{
	// Create Tiago Controller. true if gripper (steel), false if normal hand (titanium)
	TiagoBaseController *controller = new TiagoBaseController();

	// Generates the goal for Joint
	if (argc==3) {
		printf("velocity::%f e val::%f\n", atof(argv[1]), atof(argv[2]));
		controller->executeGoal(atof(argv[1]), atof(argv[2]));
		sleep(1);
		controller->executeGoal(atof(argv[1]), atof(argv[2]));
	}
	else {
		char joint[20];
		float d, a;
	
		while (argc==1) {
			cout << "Enter direction and angle value: ";
			cin >> d;
			if (d<-1)
				break;
			cin >> a;

			printf("velocity::%f e val::%f\n", d, a);
			// Set the goal
			controller->executeGoal(d, a);
		}
	}

	return 0;
}

