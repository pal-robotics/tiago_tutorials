# First Step
This tutorial makes use of the PAL Robotics Text-to-Speech action server, developed for the TiaGO robot. The tutorial uses the action server to send messages to a soundplay-node, so the first step is to obtain the sound play package. This can be done by looking at the soundplay wiki page 

<http://wiki.ros.org/sound_play>

or directly by cloning the following github package into your catkin workspace and building 

<https://github.com/ros-drivers/audio_common.git>



# tts
The tutorial consists of two versions. The main version uses a gui to allow the user to send strings for speech and visualize the feedback and result sent by the action server. To run this version launch the file without an extra argument

~~~~
$ roslaunch tts tts.launch
~~~~

The second version simply uses the terminal to display the same information and as input. To launch add the type argument

~~~~
$ roslaunch tts tts.launch type:=terminal
~~~~

Follow the prompt to send the action goal.

Refer to the wiki page for more information