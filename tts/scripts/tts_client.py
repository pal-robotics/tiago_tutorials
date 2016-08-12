#!/usr/bin/env python

import rospy
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal


if __name__ == '__main__':
	# if len(sys.argv) > 1:
	# 	text = ""
	# 	for arg in sys.argv[1:]:
	# 		text += arg + " "
	rospy.init_node('tts_client')
	client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
	rospy.loginfo("Waiting for Server")
	client.wait_for_server()
	rospy.loginfo("Reached Server")
	goal = TtsGoal()
	while not rospy.is_shutdown():
		text = raw_input("Enter sentence: ")
		if(text == "^C"):
			pass
		goal.rawtext.text = text
		client.send_goal(goal)