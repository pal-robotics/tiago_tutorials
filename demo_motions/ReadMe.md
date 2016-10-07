# Demo motions

This tutorial simply shows some of the movements possible by the TiaGO robot using the PlayMotion action server. To run the motions launch the gazebo simulator. Two models of the TiaGO robot can be used for this tutorial, namely steel and titanium.
~~~~
$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=steel
~~~~

or

~~~~
$ roslaunch tiago_gazebo tiago_gazebo.launch robot:=titanium
~~~~

The model chosen above also needs to be specified for the launching of the demo motions, as such

~~~~
$ roslaunch demo_motions motions.launch robot:=steel
~~~~

or

~~~~
$ roslaunch demo_motions motions.launch robot:=titanium
~~~~

The launch file then loads the corresponding yaml file which contains the motions for the right model type. These are then executed at random.
