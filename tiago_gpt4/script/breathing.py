#!/usr/bin/env python

import rospy
from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import time

class BreathingExercise:
    def __init__(self):
        rospy.init_node('breathing_exercise')

        self.soundhandle = SoundClient()

        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        # Wait for the sound client to properly initialize
        time.sleep(1)

    def speak(self, text):
        rospy.loginfo(text)
        self.soundhandle.say(text)
        time.sleep(2)  # Wait for speech to finish

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 3.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [self.get_current_height()]
        start_point.time_from_start = rospy.Duration(0)
        traj.points.append(start_point)

        # Target position
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_height]
        target_point.time_from_start = rospy.Duration(duration)
        traj.points.append(target_point)

        # Publish trajectory
        self.height_pub.publish(traj)
        time.sleep(duration)

    def get_current_height(self):
        # In a real implementation, you should read the current height from sensors
        # For simplicity, let's assume the initial height is 0.0
        return 0.0

    def start_exercise(self):
        self.speak("Let's take a moment to recharge and refocus. Stand tall and join me in a brief breathing exercise. It's a great way to clear your mind, relieve any tension, and come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Ready to give it a try?")

        self.speak("Find a comfortable standing position, with your feet hip-width apart and your spine straight. Close your eyes gently if you feel comfortable doing so, or simply soften your gaze.")

        for i in range(3):
            # Inhale
            self.adjust_height(0.4)  # Increase height
            self.speak("As we begin, inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale. Slowly exhale through your mouth, letting go of any stress or tension you may be holding onto.")

            time.sleep(1)  # Wait for user to hold breath

            # Exhale
            self.adjust_height(0.0)  # Decrease height
            self.speak("Again, inhale deeply through your nose, feeling the air fill your lungs. Hold briefly. Then exhale slowly and completely through your mouth, releasing any remaining tension.")

            time.sleep(1)  # Wait for user to exhale

        self.adjust_height(0.2)  # Return to default height
        self.speak("Let's do one more round, inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus. Whenever you're ready, gently open your eyes. How do you feel?")

if __name__ == '__main__':
    try:
        exercise = BreathingExercise()
        exercise.start_exercise()
    except rospy.ROSInterruptException:
        pass
