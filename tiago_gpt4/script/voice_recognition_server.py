#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import openai
import tempfile
import soundfile as sf
import sounddevice as sd
import numpy as np
import sys
import yaml
import os
from nltk.tokenize import word_tokenize
from breathing import BreathingExercise
from handover_snacks import GetSnack
from strech_ball import CatchBall


# Configure your OpenAI API key here
current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
config_path = os.path.join(current_dir, '..', 'config', 'gpt_api.yaml')  # Navigate to the config.yaml file
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
openai.api_key = config['api_key']


class VoiceRecognitionServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('voice_recognition_server', anonymous=True)
        
        # Create a publisher for the recognized text
        self.text_pub = rospy.Publisher('/tiago/recognized_text', String, queue_size=10)
        self.last_flag_timestamp = None
        self.subscriber = rospy.Subscriber('/tiago/conversation_cont', String, self.flag_callback)
        
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
        self.sample_rate = 16000 # 16000 44100
        self.threshold = 3  # Silence detection threshold
        self.silence_duration = 1  # Seconds of silence to consider the speaker has stopped
        self.stream = None
        self.last_flag_timestamp = 0
        self.first_conversation = True
        self.action_flag = False
        # self.conv_break = False
        


    def flag_callback(self, msg):
        # Update the last flag timestamp when a new message is received

        self.last_flag_timestamp = float(msg.data)

    def calibrate_threshold(self, calibration_duration=1, device_index=None):
        """Automatically calibrate the noise threshold."""
        rospy.loginfo("Calibrating microphone. Please remain silent...")
        recording = sd.rec(int(calibration_duration * self.sample_rate), samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32')
        sd.wait()  # Wait for the recording to finish
        # Calculate the RMS of the recording
        rms = np.sqrt(np.mean(np.square(recording), axis=0))
        if np.max(rms) > 0.03:
            self.threshold = np.max(rms) * 100
        else:
            self.threshold = 3  # set minimum threshold
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

    def check_grammar(self, transcript):
        """Checks and corrects the grammar of the given transcript using ChatGPT."""
        rospy.loginfo("Checking grammar")

        # Send the transcript to ChatGPT for grammar correction
        response = openai.chat.completions.create(
            model="gpt-4",
            messages= [
                {"role": "user", "content": f"Please correct the grammar and any inappropriate word of the following text: \"{transcript}\". If you think there is nothing to correct, just return 'grammatically correct'."}],
            max_tokens=500
        )
        # Access the corrected text
        # corrected_text = response['choices'][0]['message']['content'].strip() if response['choices'] else None

        # corrected_text = response.choices[0].message.content if response.choices else transcript
        corrected_text = response.choices[0].message.content
        

        # Implementing logic to return the original transcript if the correction indicates no change
        if "grammatically correct" or "Grammatically correct" in corrected_text:
            return transcript  # Return the original if the API indicates it's already correct or no meaningful correction was made
        else:
            return corrected_text  # Return the corrected text
        
        # corrected_text = response['choices'][0]['message']['content']
        # corrected_text = response.choices[0].text.strip()
        # return corrected_text



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
                # print(recorded_data[-1])
        
        self.stream = sd.InputStream(callback=callback, samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32')
        # self.stream = sd.InputStream(callback=callback, samplerate=self.sample_rate, device=device_index, dtype='float32')
        with self.stream:
            print("Recording started. Speak into the microphone.")
            while self.stream.active:
                sd.sleep(10)

        # with sd.InputStream(callback=callback, samplerate=self.sample_rate, channels=1, device=device_index, dtype='float32'):
        #     sd.sleep(5000)
        #     # sd.sleep(int(1e6))  # Wait until recording is finished based on callback

        rospy.loginfo("Recording stopped.")
        return np.concatenate(recorded_data, axis=0) 
    
    
    def recognize_speech_whisper(self):
        global categories
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
                    
                    # print(len(text))
                    # text = self.check_grammar(transcript)
                    # Check for trigger phrase

                    # current_time = rospy.get_time()
                    # # Check if the current time is within 10 seconds of the last flag timestamp
                    # if self.last_flag_timestamp is not None and current_time - self.last_flag_timestamp <= 60:
                    #     rospy.loginfo("Conversation continuous.")
                    #     corrected_text = self.check_grammar(text)
                    #     rospy.loginfo(f"You are saying: {corrected_text}")
                    #     self.text_pub.publish(corrected_text)
                    #     self.conv_break = False
                    # else:
                    #     if ("Hey" in text or "hey" in text) and "Tiago" in text:
                    #         rospy.loginfo("Trigger phrase detected.")
                    #         # Optionally, you can remove the trigger phrase from the transcript before processing
                    #         text = text.replace("Hey, Tiago", "").strip()
                    #         corrected_text = self.check_grammar(text)
                    #         rospy.loginfo(f"You are saying: {corrected_text}")
                    #         self.text_pub.publish(corrected_text)
                    #         self.conv_break = False
                    #     else:
                    #         rospy.loginfo(f"Ignoring the input. {text}")
                    #         self.conv_break = True
                    
                    
                    if (self.first_conversation == True and ("Hey" in text or "hey" in text) and "Tiago" in text) or self.first_conversation == False:
                        rospy.loginfo("Trigger phrase detected.")
                        # Optionally, you can remove the trigger phrase from the transcript before processing
                        text = text.replace("Hey, Tiago", "").strip()
                        corrected_text = self.check_grammar(text)
                        rospy.loginfo(f"You are saying: {corrected_text}")

                        # Normalize the text
                        normalized_text = corrected_text.lower()
                        # Tokenize the text
                        tokens = set(word_tokenize(normalized_text))

                        
                        found_categories = []
                        for category, keywords in categories.items():
                            for keyword in keywords:
                                if keyword.lower() in tokens:
                                    if category not in found_categories:
                                        found_categories.append(category)
                                        rospy.loginfo(f"Found category {category}")
                        
                        if len(found_categories) == 0:
                            self.text_pub.publish(corrected_text)
                            rospy.loginfo("Go to gpt")
                            # self.conv_break = False
                            self.first_conversation = False
                            self.action_flag = False
                        elif len(found_categories) != 0:
                            if found_categories[0] == "Breathing Exercises":
                                rospy.loginfo("Doing Breathing Exercises")
                                breathing = BreathingExercise()
                                breathing.run()
                                self.action_flag = True
                            elif found_categories[0] == "Get a Snack":
                                rospy.loginfo("Doing Get a Snack")
                                snake = GetSnack()
                                snake.run()
                                self.action_flag = True
                            elif found_categories[0] == "Stretch or Use a Ball":
                                rospy.loginfo("Doing Stretch or Use a Ball")
                                ball = CatchBall()
                                ball.run()
                                self.action_flag = True
                            elif found_categories[0] == "Tell a Joke":
                                rospy.loginfo("Doing Tell a Joke")
                                request_joke = "Can you tell me a Australian joke?"
                                self.text_pub.publish(request_joke)
                                self.action_flag = False
                            
                            
                    else:
                        rospy.loginfo(f"Ignoring the input. {text}")
                        # self.conv_break = True


                    # rospy.loginfo(f"Whisper thinks you said: {text}")
                    # self.text_pub.publish(text)
            except Exception as e:
                rospy.loginfo(f"Unexpected error occurred: {e}")
            except openai.BadRequestError as e:
                rospy.loginfo(f"Audio processing error: {e}")

    def run(self):
        global categories
        categories = {
            "Breathing Exercises": ["Relax", "Stressed", "Calm", "Mindful", "Unwind", "Anxiety", "Overwhelmed", "Peace", "Tranquility", "Zen", "Quiet", "Serene", "Breathe", "Pause", "Focus", "Meditate", "Chill", "Decompress", "Stillness", "Center", "Balance", "Ease", "Rest", "De-stress", "Refresh", "Tired", "Busy", "Work", "Rushed", "Deadline", "Frustrated", "Monday", "Traffic", "Noise", "Late", "Early", "Headache", "Busywork", "Multitask", "Overtime"],
            "Get a Snack": ["Hungry", "Snack", "Food", "Break", "Eat", "Treat", "Biscuit", "Cookie", "Craving", "Chocolate", "Chips", "Fruit", "Bite", "Yummy", "Refreshment", "Sweets", "Nuts", "Hungry", "Lunchtime", "Tea time", "Coffee break", "Break time", "Starving", "Sugar", "Refresh", "food"],
            "Stretch or Use a Ball": ["Exercise", "Active", "Fitness", "Workout", "Play", "Ball", "Catch", "Throw", "Sports", "Activity", "Movement", "Agility", "Fun", "Game", "Physical", "Outdoors", "Sporty", "Energize", "Dynamic", "Action", "Roll", "Toss", "Fetch", "Jump", "Run", "Sportive"],
            "Schedule a Meeting": ["Schedule", "Meeting", "Appointment", "Calendar", "Organize", "Plan", "Set up", "Arrange", "Book", "Reserve", "Video call", "Teleconference", "Zoom", "Webinar", "Date", "Time", "Slot", "Planning", "Outlook", "Google Calendar", "Reminder"],
            "Tell a Joke": ["Joke", "Laugh", "Funny", "Humor", "Comedy", "Entertainment", "Hilarious", "Sarcasm"]
        }

        while not rospy.is_shutdown():
            time_now = int(rospy.get_time())
            last_time = int(self.last_flag_timestamp)
            if self.first_conversation == True or time_now == last_time or self.action_flag == True:
                self.recognize_speech_whisper()
            else:
                # rospy.loginfo("Wait for gpt speaking")
                continue

if __name__ == '__main__':
    vr_server = VoiceRecognitionServer()
    try:
        vr_server.run()
    except rospy.ROSInterruptException:
        pass
