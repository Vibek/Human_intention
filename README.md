Track and display human motion intention using ROS, OpenCV, OpenNI and OpenGL.
=======
# Human_intention
This respository is the part of Human Intention project using Kinect contains ROS packages. The projects includes the following:

1. A modified ROS packages for human skeleton detection in both Depth and RGB frames.
2. Detection the human motion intention and using inverse kinematics to introduced left and right hand destination goal points.
3. Introduced activity based intention recognition, e.g  Drinking.
4. Defined the activity with color map like stationary  (Yellow), movement (Blue) and close to goal (Red).
5. Generated anticipate trajectory depends on current activity. In this work, we defined threshold to determined the anticipatary trajectory.
6. Save the left and right hand joint position, velocity and anticipate trajectory in csv file.
7. We visualize the skeleton and hand trajectory in RVIZ
8. We introduced attention map to visualize the predictive trajectory.
9. We proposed a algorithm which generate the attention using linear interpolate and Gaussian Blur method. 

The proposed attention map is used to visualize the motion intention, which will help to perform the tasks in more meaningful way by mobile robot.

Mobile Robot used in this work is developed by Adept Mobile Robot.
Model: SeekurJr with manipulator.


