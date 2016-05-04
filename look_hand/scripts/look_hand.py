#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 5/8/15

@author: sampfeiffer

look_hand.py contains... a class to look at
its own hand. It could be easily extended to look
at any tf link
"""
__author__ = 'sampfeiffer'

# System imports

# Local imports

# ROS imports
import rospy
import tf.transformations

# ROS messages imports
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import PointStamped

# Useful colors for prints
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

LOOK_TO_POINT_AS_GOAL_TOPIC = '/head_controller/point_head_action/goal'

class LookAtMyHand():
    """This class does stuff"""

    def __init__(self):
        # Topics
        rospy.loginfo("Setting publisher to " + LOOK_TO_POINT_AS_GOAL_TOPIC)
        self.pub_head_topic = rospy.Publisher(LOOK_TO_POINT_AS_GOAL_TOPIC, PointHeadActionGoal, queue_size=1)

        self.tf_l = tf.TransformListener()
        rospy.sleep(2.0)


    def run(self):
        r = rospy.Rate(10)
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)
        phag.goal.target.header.frame_id = "/base_link"
        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"
        while not rospy.is_shutdown():

            ps = PointStamped()
            ps.header.stamp = self.tf_l.getLatestCommonTime("/base_link", "/arm_7_link")
            ps.header.frame_id = "/arm_7_link"
            transform_ok = False
            while not transform_ok and not rospy.is_shutdown():
                try:
                    arm7link_ps = self.tf_l.transformPoint("/base_link", ps)
                    transform_ok = True
                except tf.ExtrapolationException as e:
                    rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
                    rospy.sleep(0.01)
                    ps.header.stamp = self.tf_l.getLatestCommonTime("/base_link", "/arm_7_link")
            phag.goal.target.point = arm7link_ps.point
            rospy.loginfo("Sending: " + str(phag))
            self.pub_head_topic.publish(phag)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('look_at_my_hand')
    node = LookAtMyHand()
    node.run()
    #rospy.spin()

    