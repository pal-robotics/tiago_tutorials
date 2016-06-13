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
from geometry_msgs.msg import WrenchStamped


class ForceTorquePlay(object):
    def __init__(self):
        self.last_msg = None
        self.force_torque_sub = rospy.Subscriber('/wrist_ft',
                                                 WrenchStamped,
                                                 self.force_torque_cb,
                                                 queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.force_torque_sub.resolved_name) + "' topic.")
        self.run()

    def force_torque_cb(self, msg):
        """
        :type msg: WrenchStamped
        """
        self.last_msg = msg

    def run(self):
        """Show information on what was found in the joint states current"""
        rospy.loginfo("Waiting for first WrenchStamped message...")
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_msg()
            r.sleep()

    def do_stuff_with_last_msg(self):
        """Print funny sentences about what we can guess of our status thanks to the force torque sensor"""
        # self.last_msg = WrenchStamped()  # line for autocompletion pourposes
        f = self.last_msg.wrench.force
        t = self.last_msg.wrench.torque
        total_torque = abs(t.x) + abs(t.y) + abs(t.z)
        if f.z > 10.0:
            rospy.loginfo("Looks someone is pulling from my hand!")
        elif f.z < -10.0:
            rospy.loginfo("Looks someone is pushing my hand!")
        elif total_torque > 2.0:
            rospy.loginfo("Hey, why are you twisting my hand? :(")
        else:
            rospy.loginfo(
                "Do something with my hand and I'll tell you what I feel")


if __name__ == '__main__':
    rospy.init_node('force_torque_play')
    ftp = ForceTorquePlay()
    rospy.spin()
