#!/usr/bin/env python
import rospy
from math import sin, cos, pi
from gazebo_msgs.msg import ModelState
import rospy

def animate():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rospy.init_node('animate_ball')
    rate = rospy.Rate(20) # 15 Hz
    x = 0.9
    y = 0.0
    z = 1.2
    ballState = ModelState()
    ballState.model_name = 'ball'
    ballState.pose.orientation.x = 0
    ballState.pose.orientation.y = 0
    ballState.pose.orientation.z = 0
    ballState.pose.orientation.w = 1
    ballState.reference_frame = 'world'
    direction = 'left'
    step = 0.08
    radius = 0.1
    angle = 0

    while not rospy.is_shutdown():

        ballState.pose.position.x = x
        ballState.pose.position.y = y + radius * cos(angle)
        ballState.pose.position.z = z + radius * sin(angle)

        angle = angle + step % pi        
        
        pub.publish(ballState)
        rate.sleep()

if __name__ == '__main__':
    try:
        animate()
    except rospy.ROSInterruptException:
        pass
