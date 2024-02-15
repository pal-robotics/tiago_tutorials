#!/usr/bin/env python

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def move_arm():
    rospy.init_node('tiago_arm_mover', anonymous=True)

    # Define the action client
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for arm_controller/follow_joint_trajectory server...")
    client.wait_for_server()
    rospy.loginfo("Connected to server")

    # Define the goal
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()

    # Specify the joint names
    trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    # Define the joint target positions
    point = JointTrajectoryPoint()
    point.positions = [0.21, -0.37, -1.08, 1.18, -2.07, 1.06, -1.58]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)

    # Set the trajectory in the goal
    goal.trajectory = trajectory

    # Send the goal and wait for the result
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):  # Increase timeout to ensure enough time for execution
        rospy.loginfo("Action completed successfully.")
    else:
        rospy.loginfo("Action did not complete before the timeout.")

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Interrupted before completion: %s", e)
