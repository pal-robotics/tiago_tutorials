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
from def_actions import play_action
from text_to_speech_gpt4 import TTSFunction
import time
from create_calendar import create_event_calendar
from Showing_Events_Caleder import Showing_Events_Calender
from customized_gesture import FollowMe, ShowAround


# Configure your OpenAI API key here
current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
config_path = os.path.join(current_dir, '..', 'config', 'gpt_api.yaml')  # Navigate to the config.yaml file
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
openai.api_key = config['api_key']


class VoiceRecognitionServer:
    global time_start, reminder_flag
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('voice_recognition_server', anonymous=True)
        
        # Create a publisher for the recognized text
        self.text_pub = rospy.Publisher('/tiago/recognized_text', String, queue_size=10)
        self.last_flag_timestamp = None
        self.subscriber = rospy.Subscriber('/tiago/conversation_cont', String, self.flag_callback)

        # Audio recording parameters
        self.sample_rate = 16000 # 16000 44100
        self.threshold = 3  # Silence detection threshold
        self.silence_duration = 1  # Seconds of silence to consider the speaker has stopped
        self.stream = None
        self.last_flag_timestamp = 0
        self.first_conversation = True
        self.action_flag = False
        self.speak = TTSFunction()
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
        corrected_text = response.choices[0].message.content
        

        # Implementing logic to return the original transcript if the correction indicates no change
        if "grammatically correct" or "Grammatically correct" in corrected_text:
            return transcript  # Return the original if the API indicates it's already correct or no meaningful correction was made
        else:
            return corrected_text  # Return the corrected text




    def record_until_silence(self, device_index=None):
        """Record from the microphone until silence is detected."""
        rospy.loginfo("Starting recording...")
        recorded_data = []
        silent_frames = 0
        recording = False

        def callback(indata, frames, time, status):
            nonlocal recorded_data, silent_frames, recording #, silent_frames_buffer
            if status:
                print(status, file=sys.stderr)
            amplitude = np.linalg.norm(indata)*9

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

        rospy.loginfo("Recording stopped.")
        return np.concatenate(recorded_data, axis=0) 
    
    
    def recognize_speech_whisper(self):
        global my_device_index    
        text = None
        
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
                    return text
            except Exception as e:
                rospy.loginfo(f"Unexpected error occurred: {e}")
                self.action_flag = True
            except openai.BadRequestError as e:
                rospy.loginfo(f"Audio processing error: {e}")
                self.action_flag = True


    def processing(self):
            global categories
            text = self.recognize_speech_whisper()
            if text is not None:       
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

                    follow_me = FollowMe()
                    show_around = ShowAround()

                    
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
                    else:
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

                        elif found_categories[0] == "Stress Ball":
                            rospy.loginfo("Doing Stress Ball")
                            ball = CatchBall()
                            ball.run()
                            self.action_flag = True

                        elif found_categories[0] == "Wave":
                            rospy.loginfo("Doing a wave")
                            play_action('wave')
                            play_action('home')
                            self.action_flag = True

                        elif found_categories[0] == "Tell a Joke":
                            rospy.loginfo("Doing Tell a Joke")
                            request_joke = "Can you tell me a Australian joke?"
                            self.text_pub.publish(request_joke)
                            self.action_flag = False

                        elif found_categories[0] == "Schedule a Meeting":
                            rospy.loginfo("Doing schedule a meeting")
                            create_event_calendar()
                            schedule = Showing_Events_Calender()
                            rospy.loginfo(schedule)
                            self.speak.text_to_speech(schedule, 1.2)
                            self.action_flag = True

                        elif found_categories[0] == "Navigation":
                            rospy.loginfo("Doing navigation")
                            navigation = "Which room do you want to go? Meeting room A, meeting room B, office desk, or the kitchen?"
                            self.speak.text_to_speech(navigation, 1.2)
                            time.sleep(1)

                            # listening to user's response
                            navigation_response = self.recognize_speech_whisper()
                            navigation_response = self.check_grammar(navigation_response)

                            if "a" in navigation_response or "A" in navigation_response:
                                rospy.loginfo("Show meeting room A")
                                navigation = "I'll show you meeting room A"
                                self.speak.text_to_speech(navigation, 1.2)
                                self.action_flag = True

                            elif "b" in navigation_response or "B" in navigation_response:
                                rospy.loginfo("Show meeting room B")
                                navigation = "I'll show you meeting room B"
                                self.speak.text_to_speech(navigation, 1.2)
                                follow_me.run()

                                show_around.run()
                                self.action_flag = True

                            elif "desk" in navigation_response or "Desk" in navigation_response:
                                rospy.loginfo("Show office desk")
                                navigation = "I'll show you your desk"
                                self.speak.text_to_speech(navigation, 1.2)
                                follow_me.run()

                                show_around.run()
                                self.action_flag = True

                            elif "kitchen" in navigation_response or "Kitchen" in navigation_response:
                                rospy.loginfo("Show kitchen")
                                navigation = "I'll show you our kitchen. There is a coffee machine."
                                self.speak.text_to_speech(navigation, 1.2)
                                follow_me.run()

                                show_around.run()
                                self.action_flag = True
                            else:
                                navigation = "I'm sorry I missed that, can you say it again?"
                                self.speak.text_to_speech(navigation, 1.2)
                                time.sleep(1)

                                # listening to user's response
                                navigation_response = self.recognize_speech_whisper()
                                navigation_response = self.check_grammar(navigation_response)


                                if "a" in navigation_response or "A" in navigation_response:
                                    rospy.loginfo("Show meeting room A")
                                    navigation = "I'll show you meeting room A"
                                    self.speak.text_to_speech(navigation, 1.2)
                                    follow_me.run()

                                    show_around.run()
                                    self.action_flag = True

                                elif "b" in navigation_response or "B" in navigation_response:
                                    rospy.loginfo("Show meeting room B")
                                    navigation = "I'll show you meeting room B"
                                    self.speak.text_to_speech(navigation, 1.2)
                                    follow_me.run()

                                    show_around.run()
                                    self.action_flag = True

                                elif "desk" in navigation_response or "Desk" in navigation_response:
                                    rospy.loginfo("Show office desk")
                                    navigation = "I'll show you your desk"
                                    self.speak.text_to_speech(navigation, 1.2)
                                    follow_me.run()

                                    show_around.run()
                                    self.action_flag = True

                                elif "kitchen" in navigation_response or "Kitchen" in navigation_response:
                                    rospy.loginfo("Show kitchen")
                                    navigation = "I'll show you our kitchen. There is a coffee machine."
                                    self.speak.text_to_speech(navigation, 1.2)
                                    follow_me.run()

                                    show_around.run()
                                    self.action_flag = True

                                else:
                                    navigation = "Do you want to talk somethin else?"
                                    self.speak.text_to_speech(navigation, 1.2)
                                    time.sleep(1)
                                    self.action_flag = True
                            
                        
                else:
                    rospy.loginfo(f"Ignoring the input. {text}")

            else:
                pass

            

    def run(self):
        global categories, my_device_index
        my_device_index = 7

        # Calibrate threshold before recording
        self.calibrate_threshold(device_index=my_device_index)
        categories = {
            "Stress Ball": ["Exercise", "Active", "Fitness", "Workout", "Play", "Ball", "Catch", "Throw", "Sports", "Activity", "Movement", "Agility", "Fun", "Game", "Physical", "Outdoors", "Sporty", "Energize", "Dynamic", "Action", "Roll", "Toss", "Fetch", "Jump", "Run", "Sportive"],
            "Breathing Exercises": ["Relax", "Stressed", "Calm", "Mindful", "Unwind", "Anxiety", "Overwhelmed", "Peace", "Tranquility", "Zen", "Quiet", "Serene", "Breathe", "Pause", "Focus", "Meditate", "Chill", "Decompress", "Stillness", "Center", "Balance", "Ease", "Rest", "De-stress", "Refresh", "Tired", "Busy", "Rushed", "Deadline", "Frustrated", "Traffic", "Headache"],
            "Get a Snack": ["Hungry", "Snack", "Food", "Break", "Eat", "Treat", "Biscuit", "Cookie", "Craving", "Chocolate", "Chips", "Fruit", "Bite", "Yummy", "Refreshment", "Sweets", "Nuts", "Hungry", "Lunchtime", "Tea time", "Coffee break", "Break time", "Starving", "Sugar", "Refresh", "food"],
            "Schedule a Meeting": ["Schedule", "Appointment", "Calendar", "Organize", "Plan", "Set up", "Arrange", "Book", "Reserve", "Teleconference", "Zoom", "Date", "Time", "Slot", "Planning", "Outlook", "Google Calendar", "Reminder"],
            "Navigation": {"Show", "Guide", "Where", "Navigate"},
            "Tell a Joke": ["Joke", "Laugh", "Funny", "Humor", "Comedy", "Entertainment", "Hilarious", "Sarcasm"],
            "Wave": {"Hello", "Hi"}
            # "Meeting": {"Conference", "Room", "Presentation", "Team", "Briefing", "Workshop", "Seminar", "Guide", "Meetup"}
        }

        while not rospy.is_shutdown():
            time_now = int(rospy.get_time())
            duration = time_now - time_start
            last_time = int(self.last_flag_timestamp)
            if duration > 600.0 and reminder_flag == False:
                text = "As you know, I am here for you to reduce your stress, to keep you healthy, to support you with scheduling meetings and to make your work life easier."
                self.speak.text_to_speech(text, 1)
            else:
                if self.first_conversation == True or time_now == last_time or self.action_flag == True:
                    self.processing()
                else:
                    # rospy.loginfo("Wait for gpt speaking")
                    continue

if __name__ == '__main__':
    vr_server = VoiceRecognitionServer()
    time_start = int(rospy.get_time())
    reminder_flag = False
    try:
        vr_server.run()
    except rospy.ROSInterruptException:
        pass
