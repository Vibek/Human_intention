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

//ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <kdl/frames.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/point_cloud_conversion.h>

//OpenGl libraries
#include <GL/glut.h>

//PCL Packages
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>

//basic c and c++ libraries
#include <string>
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

//Pacakge libraries design by our resporitory
#include "Human_intention/Skeleton.h"
#include "Human_intention/EnableJointGroup.h"
#include "Human_intention/Pose.h"
#include "Human_intention/Goal.h"

#include "attention_map.hpp"
#include "KinectController.h"
#include "KinectDisplay.h"
#include "trajectory.h"
#include "find_objects.h"
#include "find_objects_color.h"

#define PI 3.14159265359
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif


class SkeletonTracker
  {
    public:  

    void init();
    //~SkeletonTracker();           
    void publishTransform(KinectController &kinect_controller, XnUserID const &user, XnSkeletonJoint const &joint, string const &frame_id, string const &child_frame_id, Human_intention::Skeleton &skeleton);
    void processKinect(KinectController &kinect_controller);
    void getRobotPose(const nav_msgs::Odometry::ConstPtr& robot_pose);
    std::string fixed_frame; 

   private:

    void handtrajectory(xn::UserGenerator& userGenerator, xn::DepthGenerator& depthGenerator, XnUserID player, XnSkeletonJoint eJoint, bool updateHistory);
    bool GetHistoryForJoint (XnSkeletonJoint eJoint, History **history); 
    void enableJointGroupCB(const Human_intention::EnableJointGroupConstPtr& msg);  

    ros::Publisher  skeleton_pub_;
    ros::Publisher  marker_pub;
    ros::Publisher  robot_model_pub;  
    ros::Publisher  pose_pub; 
    ros::Publisher  torso_destination_pub_;
    ros::Publisher  left_arm_destination_pub_;
    ros::Publisher  right_arm_destination_pub_;
    ros::Publisher  cmdVelPub;
    ros::Publisher  joint_states_pub_;  
    ros::Subscriber enable_joint_group_sub_;
    ros::Subscriber robotpose_sub;
    History g_RightHandPositionHistory;
    History g_LeftHandPositionHistory;
    History *history;

  };

#endif //SKELETON_TRACKER_H
 
