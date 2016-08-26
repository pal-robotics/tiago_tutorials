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

#import System headers
import sys
import signal

#import ROS headers
import rospkg
import rospy
import actionlib

#import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal

#import GUI headers
from PyQt4 import QtGui, uic

class Action(QtGui.QMainWindow):
	def __init__(self):
		super(Action, self).__init__()
		self.rospack = rospkg.RosPack()
		path = self.rospack.get_path('tts')
		uic.loadUi(path + '/resources/text_gui.ui', self)
		
		self.client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
		rospy.loginfo("Waiting for server")

		self.client.wait_for_server()
		rospy.loginfo("Reached server")

		self.interval_slider.setMinimum(0)
		self.interval_slider.setMaximum(100)
		self.interval_slider.setValue(0)

		self.value_label.setText(str(0))
		self.interval_slider.valueChanged.connect(self.slider_cb)
		self.sendText_button.clicked.connect(self.button_cb)
		self.show()

	def feedbackCb(self, feedback):
		self.textFeedback.appendPlainText("event type: " + str(feedback.event_type))
		self.textFeedback.appendPlainText("timestamp: " + str(feedback.timestamp))
		self.textFeedback.appendPlainText("current word: " + feedback.text_said)
		self.textFeedback.appendPlainText("next word: " + feedback.next_word)


	def button_cb(self):
		self.textFeedback.clear()
		value = self.textSend.toPlainText()
		goal = TtsGoal()
		goal.rawtext.text = str(value)
		goal.rawtext.lang_id = "EN"
		goal.wait_before_speaking = float(self.interval_slider.value())/10
		self.client.send_goal(goal, feedback_cb = self.feedbackCb)
		self.client.wait_for_result()
		result = self.client.get_result()
		self.textResult.setPlainText(str(result))

	def slider_cb(self):
		self.value_label.setText(str(float(self.interval_slider.value())/10))

if __name__ == '__main__':
	rospy.init_node('tts_client_gui')
	app = QtGui.QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	action_ob = Action()
	sys.exit(app.exec_())
