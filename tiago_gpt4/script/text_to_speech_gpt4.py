#!/usr/bin/env python
# not using anymore
import rospy
from std_msgs.msg import String
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class TTSClient:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gpt4_to_tts')

        # Subscribe to the GPT-4 output topic
        self.gpt4_subscriber = rospy.Subscriber('/tiago/gpt4_response', String, self.handle_gpt4_output)

        # Connect to the text-to-speech action server
        self.tts_client = SimpleActionClient('tts_to_soundplay', TtsAction)
        rospy.loginfo("Waiting for TTS action server...")
        self.tts_client.wait_for_server()
        rospy.loginfo("TTS action server found!")

    def handle_gpt4_output(self, msg):
        # The callback function for the GPT-4 output subscriber
        text = msg.data
        rospy.loginfo(f"Received from GPT-4: {text}")
        
        # Create a goal with the received text
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"  # Assuming British English output

        # Send the goal to the TTS service and wait for the result
        rospy.loginfo("Sending text to TTS...")
        self.tts_client.send_goal_and_wait(goal)
        rospy.loginfo("TTS processing complete.")

if __name__ == '__main__':
    tts_client = TTSClient()
    rospy.spin()  # Keep the program running, listening for GPT-4 output
