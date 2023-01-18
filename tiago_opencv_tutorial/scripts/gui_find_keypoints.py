#! /usr/bin/env python

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

#System imports
import sys
import signal

#ROS imports
import rospy
import rospkg

#GUI imports
from PyQt5 import QtGui, QtWidgets, uic

#Cusotm msg imports
from tiago_opencv_tutorial.msg import valueMatrix
pub = rospy.Publisher('tiago_opencv_tutorial/find_keypoints_gui', valueMatrix, queue_size=10)
class MyWindow(QtWidgets.QMainWindow):
	def __init__(self):
		super(MyWindow, self).__init__()
		self.rospack = rospkg.RosPack()
		path = self.rospack.get_path('tiago_opencv_tutorial')
		uic.loadUi(path + '/resources/gui_keypoints.ui', self)
		self.setMinMax()
		self.setLabels()
		self.setTick()
		
		self.zero_zero_slider.valueChanged.connect(self.zero_zero_cb)
		self.zero_one_slider.valueChanged.connect(self.zero_one_cb)
		self.zero_two_slider.valueChanged.connect(self.zero_two_cb)
		self.one_zero_slider.valueChanged.connect(self.one_zero_cb)
		self.one_one_slider.valueChanged.connect(self.one_one_cb)
		self.one_two_slider.valueChanged.connect(self.one_two_cb)
		self.two_zero_slider.valueChanged.connect(self.two_zero_cb)
		self.two_one_slider.valueChanged.connect(self.two_one_cb)
		self.two_two_slider.valueChanged.connect(self.two_two_cb)
		self.alpha_slider.valueChanged.connect(self.alpha_cb)
		self.beta_slider.valueChanged.connect(self.beta_cb)
		self.Detector_choice.currentIndexChanged.connect(self.Detector_choice_cb)

		self.Contrast_tick.stateChanged.connect(self.Contrast_cb)
		self.Keypoints_tick.stateChanged.connect(self.Keypoints_cb)
		self.Original_tick.stateChanged.connect(self.Original_cb)
		self.Sharpen_tick.stateChanged.connect(self.Sharpen_cb)
		self.Combined_tick.stateChanged.connect(self.Combined_cb)

		self.show()

	def setLabels(self):
		self.zero_zero_label.setText(str(0))
		self.zero_one_label.setText(str(-1))
		self.zero_two_label.setText(str(0))
		self.one_zero_label.setText(str(-1))
		self.one_one_label.setText(str(5))
		self.one_two_label.setText(str(-1))
		self.two_zero_label.setText(str(0))
		self.two_one_label.setText(str(-1))
		self.two_two_label.setText(str(0))
		self.alpha_value_label.setText(str(2.2))
		self.beta_value_label.setText(str(50))


	def setMinMax(self):
		self.zero_zero_slider.setMinimum(-100)
		self.zero_one_slider.setMinimum(-100)
		self.zero_two_slider.setMinimum(-100)
		self.one_zero_slider.setMinimum(-100)
		self.one_one_slider.setMinimum(-100)
		self.one_two_slider.setMinimum(-100)
		self.two_zero_slider.setMinimum(-100)
		self.two_one_slider.setMinimum(-100)
		self.two_two_slider.setMinimum(-100)
		self.alpha_slider.setMinimum(0)
		self.beta_slider.setMinimum(-200)

		self.zero_zero_slider.setMaximum(100)
		self.zero_one_slider.setMaximum(100)
		self.zero_two_slider.setMaximum(100)
		self.one_zero_slider.setMaximum(100)
		self.one_one_slider.setMaximum(100)
		self.one_two_slider.setMaximum(100)
		self.two_zero_slider.setMaximum(100)
		self.two_one_slider.setMaximum(100)
		self.two_two_slider.setMaximum(100)
		self.alpha_slider.setMaximum(30)
		self.beta_slider.setMaximum(200)

		self.zero_zero_slider.setValue(00)
		self.zero_one_slider.setValue(-10)
		self.zero_two_slider.setValue(0)
		self.one_zero_slider.setValue(-10)
		self.one_one_slider.setValue(50)
		self.one_two_slider.setValue(-10)
		self.two_zero_slider.setValue(0)
		self.two_one_slider.setValue(-10)
		self.two_two_slider.setValue(0)
		self.alpha_slider.setValue(22)	
		self.beta_slider.setValue(50)	

	def setTick(self):
		self.Contrast_tick.setChecked(False)
		self.Keypoints_tick.setChecked(True)
		self.Original_tick.setChecked(False)
		self.Sharpen_tick.setChecked(False)
		self.Combined_tick.setChecked(False)

	def Detector_choice_cb(self):
		value = self.Detector_choice.currentText()
		rospy.loginfo("Called: %s", value)
		msg = valueMatrix()
		msg.header.frame_id = str(value)
		pub.publish(msg)

	def Contrast_cb(self):
		tick = self.Contrast_tick.checkState()
		self.pubTickMsg("Contrast", tick)
		rospy.loginfo("contrast: %s", tick)

	def Keypoints_cb(self):
		tick = self.Keypoints_tick.checkState()
		self.pubTickMsg("Keypoints", tick)
		rospy.loginfo("keypoints: %s", tick)

	def Original_cb(self):
		tick = self.Original_tick.checkState()
		self.pubTickMsg("Original", tick)
		rospy.loginfo("original: %s", tick)

	def Combined_cb(self):
		tick = self.Combined_tick.checkState()
		self.pubTickMsg("Combined", tick)
		rospy.loginfo("Combined: %s", tick)

	def Sharpen_cb(self):
		tick = self.Sharpen_tick.checkState()
		self.pubTickMsg("Sharpen", tick)
		rospy.loginfo("sharpen: %s", tick)								

	def zero_zero_cb(self):
		value_f = float(self.zero_zero_slider.value())
		value = float(value_f/10)
		self.zero_zero_label.setText(str(value))
		self.pubValMsg("zero_zero", value)
		rospy.loginfo("zero_zero %s", value)

	def zero_one_cb(self):
		value_f = float(self.zero_one_slider.value())
		value = float(value_f/10)
		self.zero_one_label.setText(str(value))
		self.pubValMsg("zero_one", value)
		rospy.loginfo("zero_one%s", value)

	def zero_two_cb(self):
		value_f = float(self.zero_two_slider.value())
		value = float(value_f/10)
		self.zero_two_label.setText(str(value))
		self.pubValMsg("zero_two", value)
		rospy.loginfo("zero_two%s", value)

	def one_zero_cb(self):
		value_f = float(self.one_zero_slider.value())
		value = float(value_f/10)
		rospy.loginfo("value: %s" , value)
		self.one_zero_label.setText(str(value))		
		self.pubValMsg("one_zero", value)

	def one_one_cb(self):
		value_f = float(self.one_one_slider.value())
		value = float(value_f/10)
		self.one_one_label.setText(str(value))		
		self.pubValMsg("one_one", value)
		rospy.loginfo("one_one%s", value)

	def one_two_cb(self):
		value_f = float(self.one_two_slider.value())
		value = float(value_f/10)
		self.one_two_label.setText(str(value))
		self.pubValMsg("one_two", value)
		rospy.loginfo("one_two%s", value)

	def two_zero_cb(self):
		value_f = float(self.two_zero_slider.value())
		value = float(value_f/10)
		self.two_zero_label.setText(str(value))
		self.pubValMsg("two_zero", value)
		rospy.loginfo("two_zero%s", value)

	def two_one_cb(self):
		value_f = float(self.two_one_slider.value())
		value = float(value_f/10)
		self.two_one_label.setText(str(value))
		self.pubValMsg("two_one", value)
		rospy.loginfo("two_one%s", value)

	def two_two_cb(self):
		value_f = float(self.two_two_slider.value())
		value = float(value_f/10)
		self.two_two_label.setText(str(value))
		self.pubValMsg("two_two", value)
		rospy.loginfo("two_two%s", value)
	
	def alpha_cb(self):
		value = float(self.alpha_slider.value())
		value_dec = float(value/10)
		self.alpha_value_label.setText(str(value_dec))
		self.pubValMsg("alpha", value_dec)

	def beta_cb(self):
		value = self.beta_slider.value()
		self.beta_value_label.setText(str(value))
		self.pubValMsg("beta", value)

	def pubValMsg(self, name, value):
		msg = valueMatrix()
		msg.header.frame_id = name
		msg.value = value
		pub.publish(msg)

	def pubTickMsg(self, name, tick):
		msg = valueMatrix()
		msg.header.frame_id = name
		msg.tick = tick
		pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('find_keypoints_gui')
	app = QtWidgets.QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	myWindow = MyWindow()
	sys.exit(app.exec_())
