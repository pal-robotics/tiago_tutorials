#!/usr/bin/env python

import rospy
# import sys
import actionlib
from sound_play.libsoundplay import SoundClient
from pal_interaction_msgs.msg import TtsAction, TtsFeedback, TtsResult


class TtsServer(object):
	_feedback = TtsFeedback()
	_result = TtsResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, TtsAction, self.execute_cb, auto_start = False)
		self._as.start()
		self.client = SoundClient()
		rospy.sleep(1)
		self.client.stopAll()
		rospy.loginfo("Started ActionServer")


	def execute_cb(self, goal):
		goalString = goal.rawtext.text
		rospy.loginfo(goalString)
		self.client.voiceSound(goalString).play()
		rospy.loginfo("sent string")
		self._as.set_succeeded(self._result)





if __name__ == '__main__':
	rospy.init_node('tts_to_soundplay')

	obj = TtsServer(rospy.get_name())

	rospy.spin()


