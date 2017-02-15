Recognition of human motion intention.
=======
# Intention_Recognition_Human_Robot_Interaction
This respository is the part of Human Intention project using Kinect contains ROS packages. The projects includes the following:

A modified ROS packages for human skeleton detection in both Depth and RGB frames.
Recognizing human intention based of object affordance and scene context in human-object relation.
Introduced actions prediction based of an activity is beign performed.
Defining the hand movements stationary (Yellow), moving (Blue) and close to goal (Red).
Generated anticipate trajectory based on distance preference and angular prefernce tacking into account the relative human-object psoition.
Write the data of both left and right hands joint position, both hands velocity, whole body joints position and anticipated trajectory position in to csv files.
We visualized the skeleton and hand trajectory in RVIZ.
We introduced attention map (heat-map) around the predicted trajectory w.r.t. visualize a future action.
We proposed a algorithm based on State automaton.
The software platform used in this works: ROS, OpenNI, OpenGL, OpenCV, Python, C++
We experimeted with 6 different datasets performed both in office environment and laboratory envionment in our university.
Mobile Robot used in this work is developed by Adept Mobile Robot. Model: SeekurJr with manipulator.

Due to the limited space, we can't upload the dataset. One can write an email requesting the dataset at vibek@meil.pw.edu.pl

Few datasets are uploaded in Github, you can find out https://github.com/Vibek/WUT_Datasets
