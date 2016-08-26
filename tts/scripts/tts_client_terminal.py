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

def event(event):
	return {
	1 : 'TTS_EVENT_INITIALIZATION',
	2 : 'TTS_EVENT_SHUTDOWN',
	4 : 'TTS_EVENT_SYNCHRONIZATION',
	8 : 'TTS_EVENT_FINISHED_PLAYING_UTTERANCE',
	16 : 'TTS_EVENT_MARK',
	32 : 'TTS_EVENT_STARTED_PLAYING_WORD',
	64 : 'TTS_EVENT_FINISHED_PLAYING_PHRASE',
	128 : 'TTS_EVENT_FINISHED_PLAYING_SENTENCE'
	}[event]

def feedbackCb(feedback):
	print("event type: " + event(feedback.event_type))
	print("timestamp: " + str(feedback.timestamp))
	print("current word: " + feedback.text_said)
	print("next word: " + feedback.next_word)
	print("-")

if __name__ == '__main__':

	rospy.init_node('tts_client')
	client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
	rospy.loginfo("Waiting for Server")
	client.wait_for_server()
	rospy.loginfo("Reached Server")
	goal = TtsGoal()
	while not rospy.is_shutdown():
		text = raw_input("Enter sentence: ")
		print("---")
		goal.rawtext.text = text
		goal.rawtext.lang_id = "NL"
		client.send_goal(goal, feedback_cb=feedbackCb)
		client.wait_for_result()
		res = client.get_result()
		print("text: " + res.text)
		print("warning/error msgs: " + res.msg)
		print("---")