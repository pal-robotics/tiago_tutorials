#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import openai
from text_to_speech_gpt4 import TTSFunction

# Configure your OpenAI API key here
openai.api_key = ''

class GenerationFuncion():
    def __init__(self):
        self.speak = TTSFunction()

    def process_with_gpt4(self, text):
        try:
            gpt_response = openai.chat.completions.create(
                model="gpt-4",  # Use the model identifier for GPT-4. Adjust if you're using a specific variant.
                messages=[{"role": "system", "content": "You are a helpful office assistant robot. Your name is Tiago. You are Australian. You are here to assist office work and maintain a relax vibe. You can do mindful exercise, play with stretchy ball, and tell jokes. Do not propose any other tasks."}, 
                          {"role": "user", "content": text}],
            )
            response = gpt_response.choices[0].message.content
            self.speak.text_to_speech(response, 1.0)
            rospy.loginfo(f"GPT response is: {response}")
            return response
        except Exception as e:
            rospy.logerr(f"Failed to process text with GPT-4: {e}")
            return "Error processing text with GPT-4."