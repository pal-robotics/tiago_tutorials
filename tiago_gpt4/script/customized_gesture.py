#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from text_to_speech_gpt4 import TTSFunction
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

class ShowAround:
    def __init__(self):
        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        self.speak = TTSFunction()

        # Subscribe to the torso sensor height
        self.current_height = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
        self.current_torso_height = 0.0

        self.arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("arm server connected.")
        self.gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("gripper server connected.")

        self.home_client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.home_client.wait_for_server()

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0)

    
    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 6.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]


        target_height = float(target_height)

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [self.current_torso_height]
        start_point.time_from_start = rospy.Duration(0)
        traj.points.append(start_point)

        # Target position
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_height]
        target_point.time_from_start = rospy.Duration(duration)
        traj.points.append(target_point)

        # Publish trajectory
        self.height_pub.publish(traj)
        time.sleep(duration)

    
    def move_arm(self, joint_angles, t):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.arm_client.send_goal(goal)
        if self.arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")
    
    
    def move_gripper(self, width, t):
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        # goal.command.position = width
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = width
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory
        
        self.gripper_client.send_goal(goal)
        if self.gripper_client.wait_for_result():
            rospy.loginfo("Gripper completed successfully.")
        else:
            rospy.loginfo("Gripper did not complete before the timeout.")
    
    def go_home_position(self):
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.home_client.send_goal(goal)
        self.home_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm home.")
    
    def run(self, text = None):
        # adjust height first
        self.adjust_height(0.34)
        rospy.loginfo("adjust height")
        
        # Open 
        width_open = [0.04, 0.04]
        self.move_gripper(width_open, 0.5) 

        # Arm go left
        left_joint_angles = [2.119355530845695, -1.2343393346546196, -2.9805722141531548, 1.9749106294310392, 1.5930857512282965, 0.204939238789771118, 1.2762728866684454]
        self.move_arm(left_joint_angles, 6)
        if text is not None:
            self.speak.text_to_speech(text, 1)

        right_joint_angles = [0.3410800549218427, -0.9574367897467364, -3.2205973717717713, 1.6951855224344599, 1.5293958969935455, 0.26235607404607875, 1.2431700210140577]
        self.move_arm(right_joint_angles, 4)
        rospy.loginfo("finish gesture")

        self.go_home_position()
    
class FollowMe():
    def __init__(self):
        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        self.speak = TTSFunction()

        # Subscribe to the torso sensor height
        self.current_height = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
        self.current_torso_height = 0.0

        self.arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("arm server connected.")
        self.gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("gripper server connected.")

        self.home_client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.home_client.wait_for_server()

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0)

    
    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 6.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]


        target_height = float(target_height)

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [self.current_torso_height]
        start_point.time_from_start = rospy.Duration(0)
        traj.points.append(start_point)

        # Target position
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_height]
        target_point.time_from_start = rospy.Duration(duration)
        traj.points.append(target_point)

        # Publish trajectory
        self.height_pub.publish(traj)
        time.sleep(duration)

    
    def move_arm(self, joint_angles, t):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.arm_client.send_goal(goal)
        if self.arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")
    
    
    def move_gripper(self, width, t):
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        # goal.command.position = width
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = width
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory
        
        self.gripper_client.send_goal(goal)
        if self.gripper_client.wait_for_result():
            rospy.loginfo("Gripper completed successfully.")
        else:
            rospy.loginfo("Gripper did not complete before the timeout.")
    
    def go_home_position(self):
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.home_client.send_goal(goal)
        self.home_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm home.")
    
    def run(self):
        # adjust height first
        self.adjust_height(0.34)
        rospy.loginfo("adjust height")

        # Open 
        width_open = [0, 0]
        self.move_gripper(width_open, 0.5) 

        
        # Arm go open
        open_joint_angles = [1.4782811164506282, -1.29, -2.88, 2.16, 1.63, 0.52, 1.3]
        self.move_arm(open_joint_angles, 6)

        self.speak.text_to_speech("follow me", 1)

        close_joint_angles = [1.477, -1.27, -2.88, 2.34, 1.59, -1.31, 1.29]
        self.move_arm(close_joint_angles, 1.5)

        self.move_arm(open_joint_angles, 1.5)
        self.move_arm(close_joint_angles, 1.5)
        rospy.loginfo("finish gesture")

        self.go_home_position()

if __name__ == '__main__':
    rospy.init_node('customized_gesture')
    rospy.loginfo("doing show around")
    showaround = ShowAround()
    showaround.run(text="This is our office.")
    rospy.loginfo("finish show around")

    # rospy.loginfo("doing follow me")
    # followme = FollowMe()
    # followme.run()
    # rospy.loginfo("finish follow me")
