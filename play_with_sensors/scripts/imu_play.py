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

import rospy
from sensor_msgs.msg import Imu


class ImuPlay(object):
    def __init__(self):
        self.last_msg = None
        self.imu_sub = rospy.Subscriber('/base_imu',
                                          Imu,
                                          self.imu_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.imu_sub.resolved_name) + "' topic.")
        self.run()

    def imu_cb(self, msg):
        """
        :type msg: Imu
        """
        self.last_msg = msg

    def run(self):
        """Show information on what was found in the imu"""
        rospy.loginfo("Waiting for first Imu message...")
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_msg()
            r.sleep()

    def do_stuff_with_last_msg(self):
        """Print funny sentences about what we can guess of our status thanks to the imu"""
        acc = self.last_msg.linear_acceleration
        # Ignore acc.z as it will be ~ -9.8 from gravity
        total_acc = abs(acc.x) + abs(acc.y)
        if total_acc > 1.0:
            rospy.loginfo("Oh no, stop shaking me!")
        else:
            rospy.loginfo("No one is shaking me, luckily!")


if __name__ == '__main__':
    rospy.init_node('imu_play')
    lp = ImuPlay()
    rospy.spin()
