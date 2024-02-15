#!/usr/bin/env python

import rospy
# from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
from text_to_speech_gpt4 import TTSFunction
# import openai
# import pyaudio
# from pydub import AudioSegment
# from pydub.playback import play
# import io
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
# from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

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
        # time.sleep(1)

        self.speak = TTSFunction()

        self.unfold_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.unfold_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)

        self.lower_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller/follow_joint_trajectory server...")
        self.lower_client.wait_for_server()
        rospy.loginfo("Connected to server")

    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

            # torso_height = float(torso_height.split()[0])
            # rospy.loginfo("Current Tiago torso height: %s m", self.current_torso_height)


    # def speak(self, text):
    #     rospy.loginfo(text)
    #     self.soundhandle.say(text)
    #     time.sleep(2)  # Wait for speech to finish

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 6.0  # Duration for height adjustment

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
        def speak_and_move(text, arm_action=None, height=None):
            speak_thread = threading.Thread(target=self.speak.text_to_speech, args=(text, 1.2))
            speak_thread.start()

            if arm_action == 'unfold':
                arm_thread = threading.Thread(target=self.go_unfold_arm)
                arm_thread.start()
            elif arm_action == 'lower':
                arm_thread = threading.Thread(target=self.go_lower_arm)
                arm_thread.start()

            if height is not None:
                height_thread = threading.Thread(target=self.adjust_height, args=(height,))
                height_thread.start()

            speak_thread.join()
            if arm_action is not None:
                arm_thread.join()
            if height is not None:
                height_thread.join()


        text = "Let's take a moment to recharge and refocus. Join me in a brief breathing exercise to relieve any tension, and to come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Find a comfortable standing position, with your feet hip-width apart and your spine straight."
        speak_and_move(text, height = 0.2)
        # self.speak.text_to_speech(text, 1.2)
        # text = "Find a comfortable standing position, with your feet hip-width apart and your spine straight."
        # speak_and_move(text, height=0.2)
        # self.speak.text_to_speech(text, 1.2)

        # Adjust to the defuant height
        # self.adjust_height(0.2)


        # self.speak("Let's take a moment to recharge and refocus. Stand tall and join me in a brief breathing exercise. It's a great way to clear your mind, relieve any tension, and come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Ready to give it a try?")

        # self.speak("Find a comfortable standing position, with your feet hip-width apart and your spine straight. Close your eyes gently if you feel comfortable doing so, or simply soften your gaze.")

        for i in range(2):
            # Inhale
            
            text = "Inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale."
            speak_and_move(text, 'unfold', 0.35)
            # self.speak.text_to_speech(text, 1.2)
            
            # self.go_unfold_arm()
            # self.adjust_height(0.35)  # Increase height
            
            # self.speak("As we begin, inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale. Slowly exhale through your mouth, letting go of any stress or tension you may be holding onto.")

            time.sleep(1)  # Wait for user to hold breath

            # Exhale
            
            text = "Exhale slowly and completely through your mouth, releasing any remaining tension."
            speak_and_move(text, 'lower', 0.1)
            
            # self.speak.text_to_speech(text, 1.2)

            # self.go_lower_arm()
            # self.adjust_height(0.1)  # Decrease height
            

            # self.speak("Again, inhale deeply through your nose, feeling the air fill your lungs. Hold briefly. Then exhale slowly and completely through your mouth, releasing any remaining tension.")

            time.sleep(1)  # Wait for user to exhale

        
        text = "Let's do one more round."
        speak_and_move(text)
        
        # self.speak.text_to_speech(text, 1.2)
        # self.speak("Let's do one more round, inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus. Whenever you're ready, gently open your eyes. How do you feel?")

        # Inhale
        
        text = "Inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. "
        speak_and_move(text, 'unfold', 0.35)
        
        # self.speak.text_to_speech(text, 1.2)
        # self.go_unfold_arm()
        # self.adjust_height(0.35)  # Increase height
        
        # self.speak("As we begin, inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale. Slowly exhale through your mouth, letting go of any stress or tension you may be holding onto.")

        time.sleep(1)  # Wait for user to hold breath

        # Exhale
        text = "And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus."
        speak_and_move(text, 'lower', 0.1)
        
        # self.speak.text_to_speech(text, 1.2)
        # self.go_lower_arm()
        # self.adjust_height(0.1)  # Decrease height

        text = "How do you feel?"
        speak_and_move(text, height = 0.2)
        
        # self.speak.text_to_speech(text, 1.2)
        

        # self.speak("Again, inhale deeply through your nose, feeling the air fill your lungs. Hold briefly. Then exhale slowly and completely through your mouth, releasing any remaining tension.")

        time.sleep(1)  # Wait for user to exhale

        # self.adjust_height(0.2)  # Return to default height
    
    def go_unfold_arm(self):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = [
            0.21, -0.2, -2.2, 1.15, -1.57, 0.2, 0.0  # Positions for the arm joints
        ]
        point.time_from_start = rospy.Duration(6.0)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.unfold_client.send_goal(goal)
        if self.unfold_client.wait_for_result(rospy.Duration(2.0)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Action completed successfully.")
        else:
            rospy.loginfo("Action did not complete before the timeout.")

    
    
    def go_lower_arm(self):
        
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = [
            0.21, -0.37, -1.08, 1.18, -2.07, 1.06, -1.58  # Positions for the arm joints
        ]
        point.time_from_start = rospy.Duration(6.0)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.lower_client.send_goal(goal)
        if self.lower_client.wait_for_result(rospy.Duration(2.0)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Action completed successfully.")
        else:
            rospy.loginfo("Action did not complete before the timeout.")



    '''
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
    '''
        
if __name__ == '__main__':
    try:
        exercise = BreathingExercise()
        exercise.start_exercise()
    except rospy.ROSInterruptException:
        pass
