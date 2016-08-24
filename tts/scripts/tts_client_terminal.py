#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Job van Dieten

#import ROS headers
import rospy
import actionlib

#import PAL Robotics custom headers
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
		goal.rawtext.text = text
		client.send_goal(goal)