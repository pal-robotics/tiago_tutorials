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

# System imports
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


def show_usage():
    """Show usage information giving the possible motions to use."""
    # Get the available motion names from param server
    param_names = rospy.get_param_names()
    motion_names = []
    for param_name in param_names:
        # If the parameter is like '/play_motion/motions/MOTION_NAME/joints'
        if "/play_motion/motions" in param_name and '/joints' in param_name:
            motion_name = param_name.replace('/play_motion/motions/', '')
            motion_name = motion_name.replace('/joints', '')
            motion_names.append(motion_name)

    rospy.loginfo("""Usage:

\trosrun run_motion run_motion_python_node.py MOTION_NAME"

\twhere MOTION_NAME must be one of the motions listed in: """ + str(motion_names))


def wait_for_valid_time(timeout):
    """Wait for a valid time (non-zero), this is important
    when using a simulated clock"""
    # Loop until:
    # * ros master shutdowns
    # * control+C is pressed (handled in is_shutdown())
    # * timeout is achieved
    # * time is valid
    start_time = time.time()
    while not rospy.is_shutdown():
        if not rospy.Time.now().is_zero():
            return
        if time.time() - start_time > timeout:
            rospy.logerr("Timed-out waiting for valid time.")
            exit(0)
        time.sleep(0.1)
    # If control+C is pressed the loop breaks, we can exit
    exit(0)


def get_status_string(status_code):
    return GoalStatus.to_string(status_code)

if __name__ == '__main__':
    rospy.init_node('run_motion_python')
    if len(sys.argv) < 2:
        show_usage()
        exit(0)

    rospy.loginfo("Starting run_motion_python application...")
    wait_for_valid_time(10.0)

    client = SimpleActionClient('/play_motion', PlayMotionAction)

    rospy.loginfo("Waiting for Action Server...")
    client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = sys.argv[1]
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + sys.argv[1])
    client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    action_ok = client.wait_for_result(rospy.Duration(30.0))

    state = client.get_state()

    if action_ok:
        rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
