#!/usr/bin/env python

import rospy
# from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
# from gpt4_server import GPT4Client
import openai
import pyaudio
from pydub import AudioSegment
from pydub.playback import play
import io
import time

openai.api_key = ''

class BreathingExercise:
    def __init__(self):
        rospy.init_node('breathing_exercise')

        # self.soundhandle = SoundClient()

        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        # Subscribe to the torso sensor height
        self.current_height = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
        self.current_torso_height = 0.0
        # Wait for the sound client to properly initialize
        time.sleep(1)

    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

            # torso_height = float(torso_height.split()[0])
            rospy.loginfo("Current Tiago torso height: %s m", self.current_torso_height)


    # def speak(self, text):
    #     rospy.loginfo(text)
    #     self.soundhandle.say(text)
    #     time.sleep(2)  # Wait for speech to finish

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 4.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]


        target_height = float(target_height)

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [self.current_torso_height]
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

    # def get_current_height(self):
    #     # In a real implementation, you should read the current height from sensors
    #     # For simplicity, let's assume the initial height is 0.0
    #     return self.current_height

    def start_exercise(self):
        text = "Let's take a moment to recharge and refocus. Stand tall and join me in a brief breathing exercise. It's a great way to clear your mind, relieve any tension, and come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Ready to give it a try?"
        self.text_to_speech(text)
        text = "Find a comfortable standing position, with your feet hip-width apart and your spine straight. Close your eyes gently if you feel comfortable doing so, or simply soften your gaze."
        self.text_to_speech(text)

        # Adjust to the defuant height
        self.adjust_height(0.2)


        # self.speak("Let's take a moment to recharge and refocus. Stand tall and join me in a brief breathing exercise. It's a great way to clear your mind, relieve any tension, and come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Ready to give it a try?")

        # self.speak("Find a comfortable standing position, with your feet hip-width apart and your spine straight. Close your eyes gently if you feel comfortable doing so, or simply soften your gaze.")

        for i in range(2):
            # Inhale
            
            text = "Inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale."
            self.text_to_speech(text)
            self.adjust_height(0.35)  # Increase height
            
            # self.speak("As we begin, inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale. Slowly exhale through your mouth, letting go of any stress or tension you may be holding onto.")

            time.sleep(1)  # Wait for user to hold breath

            # Exhale
            
            text = "Exhale slowly and completely through your mouth, releasing any remaining tension."
            self.text_to_speech(text)
            self.adjust_height(0.1)  # Decrease height

            # self.speak("Again, inhale deeply through your nose, feeling the air fill your lungs. Hold briefly. Then exhale slowly and completely through your mouth, releasing any remaining tension.")

            time.sleep(1)  # Wait for user to exhale

        
        text = "Let's do one more round."
        self.text_to_speech(text)
        # self.speak("Let's do one more round, inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus. Whenever you're ready, gently open your eyes. How do you feel?")

        # Inhale
        
        text = "Inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. "
        self.text_to_speech(text)
        self.adjust_height(0.35)  # Increase height
        
        # self.speak("As we begin, inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale. Slowly exhale through your mouth, letting go of any stress or tension you may be holding onto.")

        time.sleep(1)  # Wait for user to hold breath

        # Exhale
        text = "And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus."
        self.text_to_speech(text)
        self.adjust_height(0.05)  # Decrease height

        text = "Whenever you're ready, gently open your eyes. How do you feel?"
        self.text_to_speech(text)
        

        # self.speak("Again, inhale deeply through your nose, feeling the air fill your lungs. Hold briefly. Then exhale slowly and completely through your mouth, releasing any remaining tension.")

        time.sleep(1)  # Wait for user to exhale

        self.adjust_height(0.2)  # Return to default height
    
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
        
if __name__ == '__main__':
    try:
        exercise = BreathingExercise()
        exercise.start_exercise()
    except rospy.ROSInterruptException:
        pass
