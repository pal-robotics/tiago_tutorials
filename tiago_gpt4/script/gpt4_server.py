#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import openai
import threading

# Configure your OpenAI API key here
openai.api_key = ''

class GPT4Client:
    def __init__(self):
        rospy.init_node('gpt4_client', anonymous=True)
        
        # Subscriber to get data from voice recognition server
        self.subscriber = rospy.Subscriber('/tiago/recognized_text', String, self.callback)
        
        # Publisher to publish GPT-4's response
        self.publisher = rospy.Publisher('/tiago/gpt4_response', String, queue_size=10)
        
    def callback(self, data):
        recognized_text = data.data
        rospy.loginfo(f"Received recognized text: {recognized_text}")
        
        # Process the text with GPT-4
        response = self.process_with_gpt4(recognized_text)
        
        # Publish GPT-4's response
        self.publisher.publish(response)
    
    def process_with_gpt4(self, text):
        try:
            gpt_response = openai.ChatCompletion.create(
                model="gpt-4",  # Use the model identifier for GPT-4. Adjust if you're using a specific variant.
                messages=[{"role": "system", "content": "You are a helpful assistant."}, 
                          {"role": "user", "content": text}],
            )
            return gpt_response.choices[0].message['content'].strip()
        except Exception as e:
            rospy.logerr(f"Failed to process text with GPT-4: {e}")
            return "Error processing text with GPT-4."
        

def main():
    gpt4_client = GPT4Client()
    rospy.spin()

if __name__ == '__main__':
    main()