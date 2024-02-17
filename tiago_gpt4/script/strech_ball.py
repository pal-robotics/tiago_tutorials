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


class CatchBall:
    def __init__(self):
        

        self.speak = TTSFunction()

        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

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
        rospy.sleep(3.0)

        # rospy.loginfo("Connected to server")

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
    
    def speak_and_move(self, text, joint_angles, t):
        speak_thread = threading.Thread(target=self.speak.text_to_speech, args=(text, 1))
        speak_thread.start()

        arm_thread = threading.Thread(target=self.move_arm(joint_angles, t))
        arm_thread.start()

        speak_thread.join()
        arm_thread.join()
    
    def go_home_position(self):
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.home_client.send_goal(goal)
        self.home_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm home.")
    
    def run(self):
        try:

            self.adjust_height(0.19997386816795684)
            approching_joint_angles = [0.3736315590099557, 0.20883359329334705, 0.08891803726963138, 1.3141243004491554, -1.1908934103436264, 1.0312251521012576, 0.0788524943546534]
            text = "Let's do a fun activity. I'm going to grab the ball. And you have to catch it."
            self.speak_and_move(text, approching_joint_angles, 6)

            time.sleep(2)
            # Open 
            width_open = [0.2, 0.2]
            self.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box
            
            # self.move_arm(strech_joint_angles, 6)
            # Move arm to pick position
            pick_joint_angles = [0.40561548267805914, -0.3022035448741352, -0.05948123254582829, 1.3154742214480972, -1.19080077863987, 1.030710119828372, 0.07775944025032792]  # Replace with actual angles
            self.move_arm(pick_joint_angles, 6)
            # Close gripper to grasp the box
            width_close = [0.02891444858954097, 0.02891444858954097]
            self.move_gripper(width_close, 1)
            
            # Move arm to handover position
            # self.move_arm(strech_joint_angles, 6)
            
            offer_angles = [1.6202375814984122, 0.8902675775544953, 0.05220939010523248, 0.017693921090674757, 1.662789255670913, - 0.10558075854249571, 0.03211794717547409]
            text = "Go catch it!"
            self.speak_and_move(text, offer_angles, 6)
            # self.move_arm(offer_angles, 6)

            # Open 
            width_open = [0.2, 0.2]
            self.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box

            text = "Well done."
            self.speak.text_to_speech(text, 1.2)

            strech_joint_angles = [0.21, 0.35, -0.2, 0.8, -1.57, 1.37, 0.0]
            self.move_arm(strech_joint_angles, 6)

            self.go_home_position()

            # # Open gripper to release the box
            # GetSnack.move_gipper(0.1)
        except rospy.ROSInterruptException:
            pass
    

if __name__ == '__main__':
    rospy.init_node('strech_ball')
    ball = CatchBall()
    ball.run()
    # try:
    #     ball = CatchBall()

    #     ball.adjust_height(0.19997386816795684)
    #     approching_joint_angles = [0.3736315590099557, 0.20883359329334705, 0.08891803726963138, 1.3141243004491554, -1.1908934103436264, 1.0312251521012576, 0.0788524943546534]
    #     text = "Let's do a fun activity. I'm going to grab the ball. And you have to catch it."
    #     ball.speak_and_move(text, approching_joint_angles, 6)

    #     time.sleep(2)
    #     # Open 
    #     width_open = [0.2, 0.2]
    #     ball.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box
        
    #     # ball.move_arm(strech_joint_angles, 6)
    #     # Move arm to pick position
    #     pick_joint_angles = [0.40561548267805914, -0.3022035448741352, -0.05948123254582829, 1.3154742214480972, -1.19080077863987, 1.030710119828372, 0.07775944025032792]  # Replace with actual angles
    #     ball.move_arm(pick_joint_angles, 6)
    #     # Close gripper to grasp the box
    #     width_close = [0.02891444858954097, 0.02891444858954097]
    #     ball.move_gripper(width_close, 1)
        
    #     # Move arm to handover position
    #     # ball.move_arm(strech_joint_angles, 6)
        
    #     offer_angles = [1.6202375814984122, 0.8902675775544953, 0.05220939010523248, 0.017693921090674757, 1.662789255670913, - 0.10558075854249571, 0.03211794717547409]
    #     text = "Go catch it!"
    #     ball.speak_and_move(text, offer_angles, 6)
    #     # ball.move_arm(offer_angles, 6)

    #     # Open 
    #     width_open = [0.2, 0.2]
    #     ball.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box

    #     text = "Well done."
    #     ball.speak.text_to_speech(text, 1.2)

    #     strech_joint_angles = [0.21, 0.35, -0.2, 0.8, -1.57, 1.37, 0.0]
    #     ball.move_arm(strech_joint_angles, 6)

    #     ball.go_home_position()





    #     # # Open gripper to release the box
    #     # GetSnack.move_gipper(0.1)
    # except rospy.ROSInterruptException:
    #     pass