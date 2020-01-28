^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package demo_motions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2020-01-28)
------------------
* Add SYSTEM to some include_directories
* Contributors: Victor Lopez

2.0.2 (2018-11-16)
------------------

2.0.1 (2018-04-20)
------------------

2.0.0 (2018-03-20)
------------------
* Merge pull request #3 from sebastianstock/master
  Added dependencies to catkin_exported_targets
* Added dependencies to catkin_exported_targets
  The packages run_motion and demo_motions failed to build with catkin_make because of missing dependencies to PlayMotionAction. This is fixed by adding a dependeny to catkin_EXPORTED_TARGETS.
* Merge branch 'change-demo-motions' into master
  Conflicts:
  demo_motions/CMakeLists.txt
  demo_motions/ReadMe.md
  demo_motions/package.xml
  demo_motions/src/run_loop_node.cpp
* use meta data to skip planning in certain motions
* simple changes to launch file
* syntax in package.xml
* syntax Readme
* changed vector to read from yaml, now added motions will automatically be read into motion list
* Merge branch 'correct-tts-opencv-demo-motions-Readme' into 'master'
  Corrected tts opencv demo motions readme
  See merge request !9
* cleaned code and commented lines, added copyright
* corrected syntax readme files opencv, demo motions and tts
* corrected syntax readme files opencv, demo motions and tts
* corrected syntax readme files opencv, demo motions and tts
* Merge branch 'add-demo-motions' into 'master'
  Add demo motions
  See merge request !6
* renamed the launch file and added the readme file
* restructured files
* First Commit
* removed package to be added to other branch
* First commit
* Contributors: Job van Dieten, Jordi Pages, Sebastian Stock, job-1994

0.0.1 (2015-08-03)
------------------
