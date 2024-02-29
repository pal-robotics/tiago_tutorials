#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from text_to_speech_gpt4 import TTSFunction

def play_action(action_name):
  # write the name of your action as action_name, then will play that certain action.
  # support 'home', 'wave', 'shake_hands'
  client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(1.0)

  goal = PlayMotionGoal()

  if action_name == 'home':
    rospy.loginfo("home arm...")
    goal.motion_name = 'home'
    goal.skip_planning = False
  elif action_name == 'wave':
    rospy.loginfo("wave arm...")
    goal.motion_name = 'wave'
    goal.skip_planning = False
  elif action_name == 'shake_hands':
    rospy.loginfo("shake hands...")
    text = "Let's shake hands."
    TTSFunction.text_to_speech(text)
    goal.motion_name = 'shake_hands'
    goal.skip_planning = False

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(20.0))
  rospy.loginfo("Arm tucked.")


if __name__ == "__main__":
  rospy.init_node("def_action")
  # play_action('wave')
  # play_action('shake_hands')
  play_action('home')

  