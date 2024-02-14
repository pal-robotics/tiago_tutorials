#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import openai
import pyaudio
from pydub import AudioSegment
from pydub.playback import play
import io
import time

# Configure your OpenAI API key here
openai.api_key = ''

class GPT4Client:
    def __init__(self):
        rospy.init_node('gpt4_client', anonymous=True)
        
        # Subscriber to get data from voice recognition server
        self.subscriber = rospy.Subscriber('/tiago/recognized_text', String, self.callback)
        
        # Publisher to publish GPT-4's response
        self.publisher = rospy.Publisher('/tiago/gpt4_response', String, queue_size=10)
        
        # Publisher to publish a flag and timestamp
        self.flag_publisher = rospy.Publisher('/tiago/conversation_cont', String, queue_size=10)
        
        self.stream = None


    def callback(self, data):
        recognized_text = data.data
        rospy.loginfo(f"Received recognized text: {recognized_text}")
        
        # Process the text with GPT-4
        response = self.process_with_gpt4(recognized_text)

        # Publish a timestamp to indicate the conversation time
        timestamp = rospy.get_time() 
        flag_message = str(timestamp)
        self.flag_publisher.publish(flag_message)

        # Publish GPT-4's response
        self.publisher.publish(response)
    
    def process_with_gpt4(self, text):
        try:
            gpt_response = openai.chat.completions.create(
                model="gpt-4",  # Use the model identifier for GPT-4. Adjust if you're using a specific variant.
                messages=[{"role": "system", "content": "You are a helpful office assistant robot. Your name is Tiago."}, 
                          {"role": "user", "content": text}],
            )
            response = gpt_response.choices[0].message.content
            self.text_to_speech(response)
            rospy.loginfo(f"GPT response is: {response}")
            return response
        except Exception as e:
            rospy.logerr(f"Failed to process text with GPT-4: {e}")
            return "Error processing text with GPT-4."
    
    def text_to_speech(self, text):
        # 6 7 10
        my_device_index = 6

        try:
            response = openai.audio.speech.create(
                model="tts-1",
                voice="shimmer",
                input= text
                )

            audio_response = response.content
            # print(type(audio_response))
            # Convert MP3 bytes to audio segment
            audio_segment = AudioSegment.from_file(io.BytesIO(audio_response), format="mp3")

            # Export the audio segment to WAV format in memory
            wav_io = io.BytesIO()
            my_sample_rate = 44100 # 44100 16000
            audio_segment.set_frame_rate(my_sample_rate).export(wav_io, format="wav")
            wav_io.seek(0)  # Go to the beginning of the WAV BytesIO object


            # Initialize PyAudio
            p = pyaudio.PyAudio()

            # Open a stream with the proper configuration for playback
            self.stream = p.open(format=p.get_format_from_width(audio_segment.sample_width),
                            channels=audio_segment.channels,
                            rate=my_sample_rate,
                            output=True,
                            output_device_index=my_device_index)

            # Play the stream directly from the WAV BytesIO object
            wav_data = wav_io.read()
            self.stream.write(wav_data)
            print("speaking")  
            
            # Open a stream with the proper configuration for playback
            # self.stream = p.open(format=p.get_format_from_width(2),  # Assuming 16-bit PCM
            #                 channels=1,  # Assuming mono audio
            #                 rate=44100,  # Assuming a sample rate of 44100/ 16000 Hz
            #                 output=True,
            #                 output_device_index=my_device_index)

            # Play the stream directly from the bytes object
            # self.stream.write(audio_response)
            # print("speaking")

        except Exception as e:
            print(f"Failed to play audio: {e}")

        finally:
            # Close the stream
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()

        # Terminate PyAudio
        p.terminate()

def main():
    gpt4_client = GPT4Client()
    rospy.spin()

if __name__ == '__main__':
    main()