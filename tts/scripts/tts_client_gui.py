#!/usr/bin/env python
import rospy
import sys
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospkg
from PyQt4 import QtGui, uic
import signal

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
		# rospy.loginfo(feedback.text_said)
		self.textFeedback.appendPlainText("event type: " + str(feedback.event_type))
		self.textFeedback.appendPlainText("timestamp: " + str(feedback.timestamp))
		self.textFeedback.appendPlainText("current word: " + feedback.text_said)
		self.textFeedback.appendPlainText("next word: " + feedback.next_word)


	def button_cb(self):
		self.textFeedback.clear()
		value = self.textSend.toPlainTxt()
		goal = TtsGoal()
		goal.rawtext.text = str(value)
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


	# client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
	# rospy.loginfo("Waiting for Server")
	# client.wait_for_server()
	# rospy.loginfo("Reached Server")
	# goal = TtsGoal()
	# while not rospy.is_shutdown():
	# 	text = raw_input("Enter sentence: ")
	# 	goal.rawtext.text = text
	# 	client.send_goal(goal)