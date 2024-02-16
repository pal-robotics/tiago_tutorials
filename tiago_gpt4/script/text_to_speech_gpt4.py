#!/usr/bin/env python

import openai
import pyaudio
from pydub import AudioSegment
from pydub.playback import play
import io
import rospy
import yaml
import os

# Configure your OpenAI API key here
current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
config_path = os.path.join(current_dir, '..', 'config', 'gpt_api.yaml')  # Navigate to the config.yaml file
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
openai.api_key = config['api_key']


class TTSFunction:
    def __init__(self):
        self.stream = None

    def text_to_speech(self, text, my_speed):
        # 6 7 10
        my_device_index = 6

        try:
            response = openai.audio.speech.create(
                model="tts-1",
                voice="shimmer",
                input= text,
                speed=my_speed
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
            # print("speaking")  
            
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
            rospy.loginfo(f"Failed to play audio: {e}")

        finally:
            # Close the stream
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()

        # Terminate PyAudio
        p.terminate()
