#!/usr/bin/env python

import rospy
# import sys
import actionlib
from sound_play.libsoundplay import SoundClient
from pal_interaction_msgs.msg import TtsAction, TtsFeedback, TtsResult


class TtsServer(object):
	feedback = TtsFeedback()
	result = TtsResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, TtsAction, self.execute_cb, auto_start = False)
		self._as.start()
		self.client = SoundClient()
		rospy.sleep(1)
		self.client.stopAll()
		rospy.loginfo("Started ActionServer")


	def execute_cb(self, goal):
		self.feedback.event_type = 1
		self.feedback.timestamp = rospy.get_rostime()
		self._as.publish_feedback(self.feedback)

		goalString = goal.rawtext.text

		self.result.text = goalString
		self.result.msg = "This string can be used to display an eventual error or warning message during voice synthesis"
		
		rospy.sleep(goal.wait_before_speaking)
		
		words = goalString.split()
		self.sayWords(words)
		
		# self.client.say(goalString)

		self._as.set_succeeded(self.result)
		
		self.feedback.event_type = 2
		self.feedback.timestamp = rospy.get_rostime()
		self._as.publish_feedback(self.feedback)


	def sayWords(self, words):
		i = 0
		self.feedback.event_type = 32

		for word in words:
			self.client.say(word)
			self.feedback.text_said = word
			if(i<len(words)-1):
				self.feedback.next_word = words[i+1]
			else:
				self.feedback.next_word = "Reached the end of the sentence"
				self.feedback.event_type = 128
			self.feedback.timestamp = rospy.get_rostime()
			self._as.publish_feedback(self.feedback)
			i += 1
			rospy.sleep(0.7)

		del words[:]
		rospy.loginfo(str(len(words)))






if __name__ == '__main__':
	rospy.init_node('tts_to_soundplay')

	obj = TtsServer(rospy.get_name())

	rospy.spin()


