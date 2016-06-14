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
from sensor_msgs.msg import Image
import struct

# Depth image stuff from
# http://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/


def get_pixels_depth(ini_x, ini_y, end_x, end_y, depth_image=None):
    rospy.loginfo(
        "Getting average depth at: " + str((ini_x, ini_y, end_x, end_y)))
    if depth_image is None:
        rospy.loginfo("Waiting for a depth image...")
        img = rospy.wait_for_message('/xtion/depth/image_raw',
                                     Image)
    else:
        img = depth_image
    rospy.loginfo("The image has size: " + str(img.width) +
                  " width, " + str(img.height) + " height")
    if ini_x < 0:
        rospy.logerr("Can't ask for a pixel depth out of image (ini_x < 0)")
        return None
    if ini_y < 0:
        rospy.logerr("Can't ask for a pixel depth out of image (ini_y < 0)")
        return None
    if end_x > img.width:
        rospy.logerr("Can't ask for a pixel depth out of image (end_x > img.width [%s])" % (
            str(img.width)))
        return None
    if end_y > img.height:
        rospy.logerr("Can't ask for a pixel depth out of image (end_x > img.height [%s])" % (
            str(img.height)))
        return None

    if (img.step / img.width) == 4:
        rospy.loginfo("Got a rectified depth image (4 byte floats)")
    else:
        rospy.loginfo("Got a raw depth image (2 byte integers)")

    # Compute the average of the area of interest
    sum_depth = 0
    pixel_count = 0
    nan_count = 0
    for x in range(ini_x, end_x):
        for y in range(ini_y, end_y):
            pixel = get_pixel_depth(x, y, img)
            # print "Curr pixel is: '" + str(pixel) + "' of type: " +
            # str(type(pixel))
            if pixel != pixel:  # check if nan
                nan_count += 1
            else:
                sum_depth += pixel
                pixel_count += 1

    if pixel_count > 0:
        avg = sum_depth / float(pixel_count)
        rospy.loginfo("Average is: " + str(avg) + " (with " + str(pixel_count) +
                      " valid pixels, " + str(nan_count) + " NaNs)")

        return avg
    else:
        rospy.logwarn("No pixels that are not NaN, can't return an average")
        return None


def get_pixel_depth(x, y, depth_image=None):
    if depth_image is None:
        rospy.loginfo("Waiting for a depth image...")
        img = rospy.wait_for_message('/xtion/depth/image_raw',
                                     Image)
    else:
        img = depth_image

    if x < 0:
        rospy.logerr("Can't ask for a pixel depth out of image (x < 0)")
        return None
    if y < 0:
        rospy.logerr("Can't ask for a pixel depth out of image (y < 0)")
        return None
    if x > img.width:
        rospy.logerr("Can't ask for a pixel depth out of image (x > img.width [%s])" % (
            str(img.width)))
        return None
    if y > img.height:
        rospy.logerr("Can't ask for a pixel depth out of image (x > img.height [%s])" % (
            str(img.height)))
        return None

    index = (y * img.step) + (x * (img.step / img.width))

    if (img.step / img.width) == 4:
        # rospy.loginfo("Got a rectified depth image (4 byte floats)")
        byte_data = ""
        for i in range(0, 4):
            byte_data += img.data[index + i]

        distance = struct.unpack('f', byte_data)[0]
        return distance
    else:  # NOT TESTED
        rospy.logwarn("Got a raw depth image (2 byte integers) (UNTESTED) it will probably fail.")
        # Encoding 16UC1, 
        if img.is_bigendian:
            distance = (img.data[index] << 8) + img.data[index + 1]
        else:
            distance = int(img.data[index]) + (int(img.data[index + 1]) << 8)
        return distance


class RGBDPlay(object):
    def __init__(self):
        self.last_msg = None
        # To get the same kind of raw image in simulation or real robot
        if rospy.get_param('/use_sim_time', False):
            depth_32FC1_topic = '/xtion/depth/image_raw'
        else:
            depth_32FC1_topic = '/xtion/depth/image'
        self.rgbd_sub = rospy.Subscriber(depth_32FC1_topic,
                                         Image,
                                         self.depth_img_cb,
                                         queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.rgbd_sub.resolved_name) + "' topic.")
        self.run()

    def depth_img_cb(self, msg):
        """
        :type msg: Image
        """
        self.last_msg = msg

    def run(self):
        """Show information on what was found in the rgbD image"""
        rospy.loginfo("Waiting for first Image message...")
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 1Hz rate to not spam
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_msg()
            r.sleep()

    def do_stuff_with_last_msg(self):
        """Print funny sentences about what we can guess of our status thanks to the depth image"""
        avg_center_depth = self.get_center_image_depth()
        if avg_center_depth is None:
            rospy.loginfo(
                "You are so close or so far I can't see you with my depth camera!")
        elif avg_center_depth > 1.5:
            rospy.loginfo("I see you, get closer to me!")
        else:
            rospy.loginfo("I see you, I like having people close to me :)")

    def get_center_image_depth(self):
        x1 = self.last_msg.width / 2 - 10
        x2 = self.last_msg.width / 2 + 10
        y1 = self.last_msg.height / 2 - 10
        y2 = self.last_msg.height / 2 + 10

        avg_depth = get_pixels_depth(x1, y1, x2, y2, self.last_msg)
        return avg_depth


if __name__ == '__main__':
    rospy.init_node('rgbd_play')
    lp = RGBDPlay()
    rospy.spin()
