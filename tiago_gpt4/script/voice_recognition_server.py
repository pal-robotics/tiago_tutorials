#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import openai
import tempfile
import soundfile as sf
import sounddevice as sd
import numpy as np
import sys
import os
# import speech_recognition as sr

# for index, name in enumerate(sr.Microphone.list_microphone_names()):
#     print("Microphone with index {} is named \"{}\"".format(index, name))
# Configure your OpenAI API key here
openai.api_key = ''

class VoiceRecognitionServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('voice_recognition_server', anonymous=True)
        
        # Create a publisher for the recognized text
        self.text_pub = rospy.Publisher('/tiago/recognized_text', String, queue_size=10)
        
        # # Initialize the speech recognizer
        # self.recognizer = sr.Recognizer()

        # # set the microphone
        # microphone_index = 7
        # self.microphone = sr.Microphone(device_index=microphone_index)
        
        # # Adjust the recognizer sensitivity to ambient noise
        # with self.microphone as source:
        #     rospy.loginfo("Adjusting for ambient noise. Please wait...")
        #     self.recognizer.adjust_for_ambient_noise(source)
        #     rospy.loginfo("Adjustment done. Ready to recognize speech.")

        # Audio recording parameters
        self.sample_rate = 16000
        self.threshold = 0.02  # Silence detection threshold
        self.silence_duration = 0.2  # Seconds of silence to consider the speaker has stopped
        self.stream = None


    def calibrate_threshold(self, calibration_duration=2, device_index=None):
        """Automatically calibrate the noise threshold."""
        rospy.loginfo("Calibrating microphone. Please remain silent...")
        recording = sd.rec(int(calibration_duration * self.sample_rate), samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32')
        sd.wait()  # Wait for the recording to finish
        # Calculate the RMS of the recording
        rms = np.sqrt(np.mean(np.square(recording), axis=0))
        self.threshold = np.max(rms) * 100  # Set threshold to 1.5 times the RMS value
        rospy.loginfo(f"Calibration complete. New threshold: {self.threshold}")


    # def recognize_speech(self):
    #     with self.microphone as source:
    #         rospy.loginfo("Listening...")
    #         audio = self.recognizer.listen(source)
            
    #         try:
    #             # Recognize speech using Google Web Speech API
    #             text = self.recognizer.recognize_google(audio)
    #             rospy.loginfo(f"Google Web Speech thinks you said: {text}")
    #             self.text_pub.publish(text)
    #         except sr.UnknownValueError:
    #             rospy.loginfo("Google Web Speech could not understand audio")
    #         except sr.RequestError as e:
    #             rospy.loginfo(f"Could not request results from Google Web Speech service; {e}")

    def record_until_silence(self, device_index=None):
        """Record from the microphone until silence is detected."""
        rospy.loginfo("Starting recording...")
        recorded_data = []
        silent_frames = 0
        recording = False
        # silent_frames_buffer = []
        # max_silent_frames = int(self.sample_rate * self.silence_duration / 1024)  # Convert silence duration to frame count


        def callback(indata, frames, time, status):
            nonlocal recorded_data, silent_frames, recording #, silent_frames_buffer
            if status:
                print(status, file=sys.stderr)
            amplitude = np.linalg.norm(indata)*9
            # print("amplitude", amplitude)

            # is_silent = amplitude < self.threshold
            # if is_silent:
            #     silent_frames_buffer.append(1)
            # else:
            #     silent_frames_buffer.append(0)
            
            # # Only keep the most recent frames in the buffer
            # silent_frames_buffer = silent_frames_buffer[-max_silent_frames:]

            # # Check if all recent frames are silent
            # if len(silent_frames_buffer) == max_silent_frames and all(silent_frames_buffer):
            #     raise sd.CallbackStop
            # elif not is_silent:
            #     recorded_data.append(indata.copy())


            if amplitude < self.threshold:
                if recording:
                    silent_frames += 1
                if silent_frames > self.sample_rate / frames * self.silence_duration:
                    raise sd.CallbackStop
            else:
                recording = True
                silent_frames = 0
                recorded_data.append(indata.copy())
        
        self.stream = sd.InputStream(callback=callback, samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32')
        with self.stream:
            print("Recording started. Speak into the microphone.")
            while self.stream.active:
                sd.sleep(100)

        # with sd.InputStream(callback=callback, samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32'):
        #     sd.sleep(5000)
        #     # sd.sleep(int(1e6))  # Wait until recording is finished based on callback

        rospy.loginfo("Recording stopped.")
        return np.concatenate(recorded_data, axis=0) 
    
    
    def recognize_speech_whisper(self):
        my_device_index = 7

        # Calibrate threshold before recording
        self.calibrate_threshold(device_index=my_device_index)
        
        # Record audio until silence
        audio_data = self.record_until_silence(device_index = my_device_index)

        # Save audio data to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as tmpfile:
            sf.write(tmpfile, audio_data, self.sample_rate)
            
            # Open the temporary file for reading
            try:
                with open(tmpfile.name, 'rb') as audio_file:
                    # Transcribe audio file using OpenAI's Whisper model
                    transcript = openai.audio.transcriptions.create(
                        model="whisper-1",
                        file=audio_file,
                        response_format = "text",
                        language="en"
                    )
                    # Extract the transcript text
                    text = transcript
                    rospy.loginfo(f"Whisper thinks you said: {text}")
                    self.text_pub.publish(text)
            except Exception as e:
                rospy.loginfo(f"Unexpected error occurred: {e}")
            except openai.BadRequestError as e:
                rospy.loginfo(f"Audio processing error: {e}")

    def run(self):
        while not rospy.is_shutdown():
            self.recognize_speech_whisper()

if __name__ == '__main__':
    vr_server = VoiceRecognitionServer()
    try:
        vr_server.run()
    except rospy.ROSInterruptException:
        pass
