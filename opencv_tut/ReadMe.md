--First step--
clone the directory into a catkin workspace, to create a new workspace follow the following ROS official tutorial http://wiki.ros.org/catkin/Tutorials/create_a_workspace
$git clone <directory>

Make sure the terminal is in the catkin Workspace, and build the package
$catkin_make --pkg opencv_tut 


This tutorial package contains 4 seperate sections:
-Sequential Tracking
-Keypoints
-Matching
-Corner Detection



--Sequential Tracking--
This tutorial is the most simple of the 4. In this code two sequential frames are loaded from a feed and stored in the OpenCV Matrix (cv::Mat) format. These two matrices are compared using the OpenCV absdiff method. The areas that are not the same are shown by a white region. This is useful for tracking moving objects with a simple method. To run:

On the MASTER URI, launch tiago gazebo simulation in any world, the default is empty
$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=steel

On any ROS system connected to the MASTER URI, run the following program
$ rosrun opencv_tut track_sequential

In gazebo, create a sphere from the toolbar, resize it to be roughly between the size of a football to a tennisball. Drag the ball in front of and above the camera, and release the ball.
----



--Keypoints--
This tutorial covers the multiple methods for detecting keypoints in an image. It also includes two different basic image processing methods for sadjusting the sharpness and contrast of the image. This tutorial comes with a Graphic User Interface to allow the user to change variables during runtime.

Firstly, on the MASTER URI launch tiago gazebo in the tutorial office world
$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=titanium world:=tutorial_office

Then, on any ROS System connected to the MASTER URI, launch the following file
$ roslaunch opencv_tut keypoint_tutorial.launch

Use the Gui to change the varibales and see the different keypoint detectors. See the Wiki page for more detail.
----
