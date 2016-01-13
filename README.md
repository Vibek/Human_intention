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


