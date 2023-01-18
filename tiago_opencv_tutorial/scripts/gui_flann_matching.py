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

#Custom msg imports
from tiago_opencv_tutorial.msg import valueMatrix

pub = rospy.Publisher('tiago_opencv_tutorial/flann_matching_gui', valueMatrix, queue_size=10)

class MyWindow(QtWidgets.QMainWindow):
	def __init__(self):
		super(MyWindow, self).__init__()
		self.rospack = rospkg.RosPack()
		path = self.rospack.get_path('tiago_opencv_tutorial')
		uic.loadUi(path + '/resources/gui_flann_matching.ui', self)
		self.setMinMax()
		self.setLabels()
		self.setTick()
		self.k_slider.valueChanged.connect(self.k_cb)
		self.dist_slider.valueChanged.connect(self.dist_cb)
		self.feature_choice.currentIndexChanged.connect(self.feature_choice_cb)
		self.extractor_choice.currentIndexChanged.connect(self.extractor_choice_cb)
		self.matcher_choice.currentIndexChanged.connect(self.matcher_choice_cb)
		self.knnMatching_check.stateChanged.connect(self.knn_cb)
		self.use_button.clicked.connect(self.use_button_cb)
		self.show()

	def setLabels(self):
		self.dist_label_var.setText(str(0.5))
		self.k_label_var.setText(str(2))

	def setMinMax(self):
		self.dist_slider.setMinimum(0)
		self.dist_slider.setMaximum(10)
		self.k_slider.setMinimum(1)
		self.k_slider.setMaximum(10)
		self.dist_slider.setValue(6)
		self.k_slider.setValue(2)

	def setTick(self):
		self.knnMatching_check.setChecked(False)
	
	def use_button_cb(self):
		fname = QtWidgets.QFileDialog.getOpenFileName(self, 'Open file',"/tiago_public_ws","Image files (*.jpg *.gif *.png)")[0]
		self.pubString("path", str(fname))

	def knn_cb(self):
		tick = self.knnMatching_check.checkState()
		self.pubTickMsg("knn", tick)
		rospy.loginfo("knn: %s", tick)
	
	def feature_choice_cb(self):
		value = self.feature_choice.currentText()
		self.pubString("feature_choice", str(value))

	def matcher_choice_cb(self):
		value = self.matcher_choice.currentText()
		self.pubString("matcher_choice", str(value))

	def extractor_choice_cb(self):
		value = self.extractor_choice.currentText()
		self.pubString("extractor_choice", str(value))

	def dist_cb(self):
		value = float(self.dist_slider.value())
		value_dec = float(value/10)
		self.dist_label_var.setText(str(value_dec))
		self.pubValMsg("dist", value_dec)

	def k_cb(self):
		value = self.k_slider.value()
		self.k_label_var.setText(str(value))
		self.pubValMsg("k", value)

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

	def pubString(self,name, string):
		msg = valueMatrix()
		msg.header.frame_id = name
		msg.option = string
		pub.publish(msg)
		
if __name__ == '__main__':
	rospy.init_node('flann_matching_gui')
	app = QtWidgets.QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	myWindow = MyWindow()
	sys.exit(app.exec_())
