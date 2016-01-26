/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef SKELETON_TRACKER_H
#define SKELETON_TRACKER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <skeleton_markers/Skeleton.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#include "KinectController.h"
#include "KinectDisplay.h"
#include "trajectory.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <fstream>
#include <stdio.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <cmath>
#include <math.h>/* acos*/
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <string>
#include <skeleton_markers/EnableJointGroup.h>
#include <skeleton_markers/Pose.h>
#include <skeleton_markers/Goal.h>
#include <skeleton_markers/Markers.h>

class SkeletonTracker
  {
    public:
           
      SkeletonTracker();
      ~SkeletonTracker();
      
      std::string fixed_frame;
      Human_intention::Pose odomPose;
      nav_msgs::Odometry robot_pose;
      void publishTransform(KinectController &kinect_controller, XnUserID const &user, XnSkeletonJoint const &joint, string const &frame_id, string const &child_frame_id, skeleton_markers::Skeleton &skeleton);
      void processKinect(KinectController &kinect_controller);
   
   private:
    void enableJointGroupCB(const skeleton_markers::EnableJointGroupConstPtr& msg);
    ros::Publisher  skeleton_pub_;
    ros::Publisher  marker_pub;    
    ros::Publisher  torso_destination_pub_;
    ros::Publisher  left_arm_destination_pub_;
    ros::Publisher  right_arm_destination_pub_;
    ros::Publisher  cmdVelPub;
    ros::Publisher  joint_states_pub_;  
    ros::Subscriber  enable_joint_group_sub_;
       
  };

#endif //SKELETON_TRACKER_H
 
