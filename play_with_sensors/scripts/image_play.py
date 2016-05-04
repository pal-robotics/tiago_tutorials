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
#   * Sammy Pfeiffer

from PyQt4 import QtGui, QtCore
import rospy
from sensor_msgs.msg import CompressedImage


class Main(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(Main, self).__init__(parent)
        self.image_label = QtGui.QLabel()
        self.image_label.resize(640, 480)
        self.pixmap = QtGui.QPixmap()

        self.layout = QtGui.QHBoxLayout()
        self.layout.addWidget(self.image_label)
        self.central_widget = QtGui.QWidget()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.last_msg = None
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw/compressed',
                                          CompressedImage,
                                          self.image_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.image_sub.resolved_name) + "' topic.")

        # Use Qt timers to fire up the update of image
        # as the ROS callback is in another thread and we can't do it from there
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.show_image)
        self.timer.start(1.0 / 30.0)  # show 30hz image

    def image_cb(self, msg):
        self.last_msg = msg

    def show_image(self):
        if self.last_msg is not None:
            self.pixmap.loadFromData(self.last_msg.data)
            self.image_label.setPixmap(self.pixmap)
            self.image_label.adjustSize()

if __name__ == '__main__':
    rospy.init_node('image_play')
    app = QtGui.QApplication(["image_play window"])
    myWidget = Main()
    myWidget.show()
    app.exec_()
