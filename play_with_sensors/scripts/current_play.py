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
from sensor_msgs.msg import JointState


class CurrentPlay(object):
    def __init__(self):
        self.last_msg = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                          JointState,
                                          self.joint_states_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.js_sub.resolved_name) + "' topic.")
        self.run()

    def joint_states_cb(self, msg):
        """
        :type msg: JointState
        """
        self.last_msg = msg

    def run(self):
        """Show information on what was found in the joint states current"""
        rospy.loginfo("Waiting for first JointState message...")
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_msg()
            r.sleep()

    def do_stuff_with_last_msg(self):
        """Print funny sentences about what we can guess of our status thanks to the currents read"""
        currents_sum = sum(self.last_msg.effort)
        rospy.loginfo("Looks like we are consuming " + str(currents_sum) + " ampers with our motors!")


if __name__ == '__main__':
    rospy.init_node('current_play')
    cp = CurrentPlay()
    rospy.spin()
