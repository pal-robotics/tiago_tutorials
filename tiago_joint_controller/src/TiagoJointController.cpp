// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

#include "TiagoJointController.h"

/**
 *  TiagoJointController for TIAGo robot.
 *  
 *  @author derzu
 *
 *  Compile isolated this file :
 *  g++ TiagoJointController.cpp -o TiagoJointController -I/opt/ros/indigo/include -I../include/ -I. -Iinclude/ -L/opt/ros/indigo/lib -Wl,-rpath,/opt/ros/indigo/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib
 *
 *  If you dont know the joints and the possible values test with the rqt_joint_trajectory_controller:
 *  $ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
 * 
 *  To run this controller with ROSRun:
 *  $ rosrun tiago_joint_controller TiagoJointController
 **/


TiagoJointController::TiagoJointController(bool gripper) {
	this->gripper = gripper;
	
	init();
}


TiagoJointController::TiagoJointController() {
	gripper = false; // default is the Hand.	
	
	init();
}

/**
 * Init the ROS node, clients controllers and goals.
 **/
void TiagoJointController::init() {
	int argc=1;
	char name[] = "TiagoJointController";
	char * argv = (char *) name;
	
	// Init the ROS node
	ros::init(argc, &argv, "TiagoJointController");

	ROS_INFO("Starting TiagoJointController");

	n = new ros::NodeHandle();

	// Precondition: Valid clock
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
	{
		ROS_FATAL("Timed-out waiting for valid time.");
	}

	// Create an controllers clients to move the TIAGo's joints
	createClients();

	goals[ARM]   = initArmGoal();
	goals[TORSO] = initTorsoGoal();
	goals[HEAD]  = initHeadGoal();
	if (gripper)
		goals[HAND] = initGripperGoal();
	else
		goals[HAND] = initHandGoal();
		
	
	
	lastController = ARM;
	sendAll = false;
}

TiagoJointController::~TiagoJointController() {
	if (n)
		delete n;
}


/**
 * Init the Arm Goal with 7 joints.
 * All the joints are set to initial value 0.
 **/
control_msgs::FollowJointTrajectoryGoal TiagoJointController::initArmGoal() {
	control_msgs::FollowJointTrajectoryGoal goal;

	// The joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("arm_1_joint");
	goal.trajectory.joint_names.push_back("arm_2_joint");
	goal.trajectory.joint_names.push_back("arm_3_joint");
	goal.trajectory.joint_names.push_back("arm_4_joint");
	goal.trajectory.joint_names.push_back("arm_5_joint");
	goal.trajectory.joint_names.push_back("arm_6_joint");
	goal.trajectory.joint_names.push_back("arm_7_joint");

	initGoal(goal);

	return goal;
}

/**
 * Init the Head Goal with 2 joints.
 * All the joints are set to initial value 0.
 **/
control_msgs::FollowJointTrajectoryGoal TiagoJointController::initHeadGoal() {
	control_msgs::FollowJointTrajectoryGoal goal;

	// The joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("head_1_joint");
	goal.trajectory.joint_names.push_back("head_2_joint");

	initGoal(goal);

	return goal;
}

/**
 * Init the Hand Goal with 3joints.
 * All the joints are set to initial value 0.
 **/
control_msgs::FollowJointTrajectoryGoal TiagoJointController::initHandGoal() {
	control_msgs::FollowJointTrajectoryGoal goal;

	// The joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("hand_index_joint");
	goal.trajectory.joint_names.push_back("hand_mrl_joint");
	goal.trajectory.joint_names.push_back("hand_thumb_joint");

	initGoal(goal);

	return goal;
}

/**
 * Init the Gripper Goal with 2 joints.
 * All the joints are set to initial value 0.
 **/
control_msgs::FollowJointTrajectoryGoal TiagoJointController::initGripperGoal() {
	control_msgs::FollowJointTrajectoryGoal goal;

	// The joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
	goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

	initGoal(goal);

	return goal;
}

/**
 * Init the Torso Goal with 1 joints.
 * The joints are set to initial value 0.
 **/
control_msgs::FollowJointTrajectoryGoal TiagoJointController::initTorsoGoal() {
	control_msgs::FollowJointTrajectoryGoal goal;

	// The joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("torso_lift_joint");

	initGoal(goal);

	return goal;
}

/**
 * Set all the joints of the goal to 0 and speed to 0.5.
 **/
void TiagoJointController::initGoal(control_msgs::FollowJointTrajectoryGoal &goal) {
	int size = goal.trajectory.joint_names.size();
	//printf("size::%d\n", size);

	// One waypoints in this goal trajectory
	goal.trajectory.points.resize(1);

	// First trajectory point
	int index = 0;
        // Positions e Velocities
	goal.trajectory.points[index].positions.resize(size);
	goal.trajectory.points[index].velocities.resize(size);
	for (int j = 0; j < size; ++j)
	{
		goal.trajectory.points[index].positions[j] = 0.0;
		goal.trajectory.points[index].velocities[j] = 0.0;
	}
	
	// To be reached 2 second after starting along the trajectory
	goal.trajectory.points[index].time_from_start = ros::Duration(1.0);
}


// Create a ROS action client to move TIAGo's arm
/**
 * Create 4 ROS action client, one for each controller type.
 **/
void TiagoJointController::createClients()
{
	ROS_INFO("Creating action clients");

	clients[ARM].reset(   new control_client("/arm_controller/follow_joint_trajectory") );
	clients[TORSO].reset( new control_client("/torso_controller/follow_joint_trajectory") );
	clients[HEAD].reset(  new control_client("/head_controller/follow_joint_trajectory") );
	if (gripper)
		clients[HAND].reset( new control_client("/gripper_controller/follow_joint_trajectory") );
	else
		clients[HAND].reset( new control_client("/hand_controller/follow_joint_trajectory") );

	int iterations = 0, max_iterations = 3;
	// Wait for arm controller action server to come up
	while( !clients[0]->waitForServer(ros::Duration(2.0)) && !clients[1]->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
	{
		ROS_DEBUG("Waiting for the controller_action server to come up");
		++iterations;
	}

	if ( iterations == max_iterations )
		throw std::runtime_error("Error in createClients: arm controller action server not available");
		
	// The cmd_put is not being used.
	cmd_pub[ARM]   = n->advertise<trajectory_msgs::JointTrajectory>("arm_controller/command",100);
	cmd_pub[TORSO] = n->advertise<trajectory_msgs::JointTrajectory>("torso_controller/command",100);
	cmd_pub[HEAD]  = n->advertise<trajectory_msgs::JointTrajectory>("head_controller/command",100);
	if (gripper)
		cmd_pub[HAND] = n->advertise<trajectory_msgs::JointTrajectory>("gripper_controller/command",100);
	else
		cmd_pub[HAND] = n->advertise<trajectory_msgs::JointTrajectory>("hand_controller/command",100);
		
	ros::Rate loop_rate(10);
}


/**
 * Set the especif goal for the joint with the value.
 * 
 * @param joint string name of the specific joint
 * @param value raw value of the joint. It's usually radian degrees.
 *
 **/
void TiagoJointController::setGoal(const char * joint, float value) {
	int intJoint=0;
	int intGoal = -1;
	
	// arm_1_joint
	// arm_2_joint
	// arm_3_joint
	// arm_4_joint
	// arm_5_joint
	// arm_6_joint
	if ( strncmp(joint, "arm_", 4)==0 && strlen(joint)>4) {
		intGoal = ARM;
		intJoint = joint[4] - '0';
	}
	
	// head_1_joint
	// head_2_joint
	else if ( strncmp(joint, "head_", 5)==0 && strlen(joint)>5) {
		intGoal = HEAD;
		intJoint = joint[5] - '0';
	}
	
	// hand_index_joint
	// hand_mrl_joint
	// hand_thumb_joint
	else if ( strncmp(joint, "hand_", 5)==0 && strlen(joint)>6) {
		intGoal = HAND;
		joint += 5; // shift 5 the joint char vector
		if (strncmp(joint, "index", 5)==0)
			intJoint = 1;
		else if (strncmp(joint, "mrl", 3)==0)
			intJoint = 2;
		else if (strncmp(joint, "thumb", 5)==0)
			intJoint = 3;
	}
	
	// gripper_left_finger_joint
	// gripper_right_finger_joint
	else if ( strncmp(joint, "gripper_", 8)==0 && strlen(joint)>9) {
		intGoal = HAND;
		joint += 8; // shift 8 the joint char vector
		if (strncmp(joint, "left", 4)==0)
			intJoint = 1;
		else if (strncmp(joint, "right", 5)==0)
			intJoint = 2;
	}
	
	// torso_lift_joint
	else if ( strncmp(joint, "torso_lift", 10)==0 ) {
		intGoal = TORSO;
		intJoint = 1; // always 1
	}
	
	lastController = intGoal;
	//printf("goal=%d::joint::%d\n", intGoal, intJoint);

	if (intGoal==-1)
		printf("Invalid Controller::%d\n", intGoal);
	else if (intJoint<1 || intJoint>7) {
		printf("Invalid Joint::%d\n", intJoint);
	}
	else
		goals[intGoal].trajectory.points[0].positions[intJoint-1] = value;
}


/**
 * Excute the goals of one or all the clients
 **/
void TiagoJointController::execute(bool sendAll) {
	if (sendAll) // default is false.
		for (int i=0 ; i< TOT ; i++) {
			printf("Enviando goal %d\n", i);
			// Sends the command to start the given trajectory 1s from now
			goals[i].trajectory.header.stamp = ros::Time(0);//ros::Time::now();// + ros::Duration(1.0);
			clients[i]->sendGoal(goals[i]);
		}
	else if (lastController>=0) {
		// Sends the command to start the given trajectory 1s from now
		goals[lastController].trajectory.header.stamp = ros::Time(0);//ros::Time::now();// + ros::Duration(1.0);
		
		//cmd_pub[lastController].publish(goals[lastController].trajectory);
		//ros::spinOnce();

		clients[lastController]->sendGoal(goals[lastController]);
		
		// Wait for trajectory execution
		/*while( !clients[lastController]->getState().isDone() && ros::ok())
		{
			printf("Dormindo goal \n");
			ros::Duration(1).sleep(); // sleep for four seconds
		}*/
	}
}


/**
 * Excute the goals of one client
 **/
void TiagoJointController::execute() {
	execute(false); // default is false
}

using namespace std;

/**
 * Test the TiagoJointController
 * Rename mainTest to main, to test just this file.
 **/
int main(int argc, char** argv)
{
	// Create Tiago Controller. true if gripper (steel), false if normal hand (titanium)
	TiagoJointController *controller = new TiagoJointController(false);

	// Generates the goal for Joint
	if (argc==3) {
		controller->setGoal(argv[1], atof(argv[2]));
		controller->execute();
	}
	else {
		printf("Usage examples:\n\tarm_1_joint 1.55\n\thead_2_joint 1.0\n\ttorso_lift_joint 0.33\n\thand_index_joint 1\n\tgripper_right_joint 0\n\tquit\n\n");
		char joint[20];
		float f;
	
		controller->setGoal("torso_lift", 0.33); // sobe o torso
		controller->execute();
		sleep(2);
		// Home position.
		controller->setGoal("arm_2_joint", -1.34); // gira o braco para baixo
		controller->setGoal("arm_3_joint", -0.2); // gira o braco para fora
		controller->setGoal("arm_4_joint", 1.96); // antebraco em 90
		controller->setGoal("arm_5_joint", -1.56); //
		controller->setGoal("arm_6_joint", 1.33); //
		controller->execute();
	
		while (argc==1) {
			cout << "Enter joint and value: ";
			cin >> joint;
			if (strcmp(joint, "quit")==0)
				break;
			cin >> f;

			//printf("joint::%s e val::%f\n", joint, f);
			// Set the goal
			controller->setGoal(joint, f);
				
			controller->execute();
		}
	}

	return 0;
}













