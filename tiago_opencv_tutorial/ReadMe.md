# First step
Create a workspace through the following tutorial

<http://wiki.ros.org/catkin/Tutorials/create_a_workspace>

Then clone the following link into the catkin work directory

`$ git clone <directory>`

Make sure the terminal is in the catkin Workspace, and build the package

`$ catkin_make --pkg tiago_opencv_tutorial`

These tutorials require speciffically OpenCV 2.4 to be installed. If this is not yet the case, follow the instructions on the OpenCV page, such as 

<http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html>

An extra module needed is the libopencv-nonfree-dev. If it is not yet installed follow these steps

~~~~
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update
sudo apt-get install libopencv-nonfree-dev
~~~~

**This tutorial package contains 4 separate sections**
- Sequential Tracking
- Keypoints
- Matching
- Corner Detection


# Sequential Tracking
This tutorial is the most simple of the 4. In this code two sequential frames are loaded from a feed and stored in the OpenCV Matrix (cv::Mat) format. These two matrices are compared using the OpenCV absdiff method. The areas that are not the same are shown by a white region. This is useful for tracking moving objects with a simple method. To run:

On the MASTER URI, launch tiago gazebo simulation in any world, the default is empty

` $ roslaunch tiago_gazebo tiago_gazebo.launch robot:=steel `

On any ROS system connected to the MASTER URI, run the following program

` $ rosrun tiago_opencv_tutorial track_sequential `

In gazebo, create a sphere from the toolbar, resize it to be roughly between the size of a football to a tennisball. Drag the ball in front of and above the camera, and release the ball.



# Keypoints
This tutorial covers the multiple methods for detecting keypoints in an image. It also includes two different basic image processing methods for sadjusting the sharpness and contrast of the image. This tutorial comes with a Graphical User Interface (GUI) to allow the user to change variables during runtime.

Firstly, on the MASTER URI launch tiago gazebo in the tutorial office world

`$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=titanium world:=tutorial_office`

Then, on any ROS System connected to the MASTER URI, launch the following file

`$ roslaunch tiago_opencv_tutorial keypoint_tutorial.launch`

Use the Gui to change the varibales and see the different keypoint detectors. See the Wiki page for more detail. Use a teleop to move around the office, a recommended place is in front of the wall with the image of the PAL Robotics REEM robot, as this offers multiple features, and will also be used for the matching section of this package.



# Matching
Matching takes two images, one from the feed, and another static image to compare. the keypoints are found in each image, and then the data structures that hold these keypoints are compared using a matcher. The GUI offers the ability to switch between most types of Keypoint Detector and matcher available in OpenCV 2.4.

As in previous tutorials, first launch tiago gazebo in the office tutorial world on the MASTER URI

`$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=titanium world:=tutorial_office`

Then launch the matching tutorial launch file on any system connected to MASTER URI

`$ roslaunch tiago_opencv_tutorial matching_tutorial.launch`

Use the gui to swtich between the matchers and detectors. More information can be found on the wiki page.



# Corner Detection
This code allows the user to determine the quality of two types of corner detectors, ShiTomasi and Harris.

As in previous tutorials, first launch tiago gazebo in the office tutorial world on the MASTER URI

`$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=titanium world:=tutorial_office`

execute the corner detection node on any system connected to the MASTER URI

`$ rosrun tiago_opencv_tutorial corner_detection`

It is recommended to use a teleop software to move the robot in front of the cupboard in the corner of the office, as this provides nice corners to test the algorithms on. Use the sliders in the window to change the variables that the corner detection algorithms use. 
