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

# To force division to be float
from __future__ import division
import rospy
from sensor_msgs.msg import LaserScan


class LaserPlay(object):
    def __init__(self):
        self.last_scan = None
        self.laser_sub = rospy.Subscriber('/scan',
                                          LaserScan,
                                          self.scan_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.laser_sub.resolved_name) + "' topic.")
        self.run()

    def scan_cb(self, msg):
        """
        :type msg: LaserScan
        """
        self.last_scan = msg

    def run(self):
        """Show information on what was found in the laser scans"""
        rospy.loginfo("Waiting for first LaserScan message...")
        while not rospy.is_shutdown() and self.last_scan is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_scan()
            r.sleep()

    def do_stuff_with_last_scan(self):
        """Print funny sentences about what we can guess of our surroundings by the laser"""

        close_reads = 0
        far_reads = 0
        for r in self.last_scan.ranges:
            # If reading is under 0.5 meters
            if r < 1.0:
                close_reads += 1
            # if reading is over 2.0 meters
            elif r > 3.0:
                far_reads += 1

        # if 30%+ reads are close
        if close_reads / len(self.last_scan.ranges) > 0.5:
            rospy.loginfo("Looks like I'm surrounded of friends :D")

        # if 30% reads are far
        if far_reads / len(self.last_scan.ranges) > 0.5:
            rospy.loginfo("Looks like I'm alone :(")


if __name__ == '__main__':
    rospy.init_node('laser_play')
    lp = LaserPlay()
    rospy.spin()
