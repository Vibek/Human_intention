/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <skeleton_markers/Skeleton.h>
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

#define PI 3.14159265359
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

bool publish_kinect_tf_ = false;
bool right_arm_enabled_ = false; 
bool left_arm_enabled_ = false; 
bool legs_enabled_=false; 
bool motion_enabled_=false;
      
using namespace std;
using std::string;
using geometry_msgs::Twist;

Twist  vel;

xn::Context	g_context;
xn::ScriptNode	g_scriptNode;

namespace skeleton_tracker
{
 class SkeletonTracker
  {
    public:
      std::string fixed_frame;
      
      SkeletonTracker()
      {    
      }
      
      void init()
      {
        ros::NodeHandle n;
        ros::NodeHandle np("~");
	    int rate;
	    n.param("tracking_rate", rate, 1);
	    n.param("fixed_frame", fixed_frame, std::string("openni_depth_frame"));
            skeleton_pub_ = n.advertise<skeleton_markers::Skeleton>("/skeleton", rate);
            enable_joint_group_sub_ = n.subscribe("enable_joint_group", 10,
                                        &SkeletonTracker::enableJointGroupCB, this);
            np.param<bool>("publish_kinect_tf", publish_kinect_tf_, false);  
            np.param<bool>("force_left_arm_enabled", left_arm_enabled_, false);
            np.param<bool>("force_right_arm_enabled", right_arm_enabled_, false);
            np.param<bool>("force_legs_enabled", legs_enabled_, false);
            np.param<bool>("motion_enabled", motion_enabled_, false); 
            joint_states_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
            left_arm_destination_pub_ = n.advertise<geometry_msgs::Point>("left_arm_destination", 1);
            right_arm_destination_pub_ = n.advertise<geometry_msgs::Point>("right_arm_destination", 1);	
            cmdVelPub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);	
            torso_destination_pub_ = n.advertise<geometry_msgs::Point>("torso_destination", 1);
                        
      }
      
      void publishTransform(KinectController &kinect_controller, XnUserID const &user, XnSkeletonJoint const &joint, string const &frame_id, string const &child_frame_id, skeleton_markers::Skeleton &skeleton)
      {

      	xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        static tf::TransformBroadcaster br;

        XnSkeletonJointPosition joint_position;
        UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
       
        double x = joint_position.position.X / 1000.0;
        double y = joint_position.position.Y / 1000.0;
        double z = joint_position.position.Z / 1000.0;

        XnSkeletonJointOrientation joint_orientation;
        UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

        XnFloat* m = joint_orientation.orientation.elements;
        KDL::Rotation rotation(m[0], m[1], m[2],
			       m[3], m[4], m[5],
			       m[6], m[7], m[8]);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);

		geometry_msgs::Vector3 position;
		geometry_msgs::Quaternion orientation;

		position.x = x;
		position.y = y;
		position.z = z;

		orientation.x = qx;
		orientation.y = qy;
		orientation.z = qz;
		orientation.w = qw;

		skeleton.name.push_back(child_frame_id);
		skeleton.position.push_back(position);
		skeleton.orientation.push_back(orientation);
		skeleton.confidence.push_back(joint_position.fConfidence);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));

        tf::Transform change_frame;
        change_frame.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion frame_rotation;
        frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
        change_frame.setRotation(frame_rotation);

        transform = change_frame * transform;

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
      }

//History function define in trajectory.h (file)
History g_RightHandPositionHistory;
History g_LeftHandPositionHistory;

bool GetHistoryForJoint (XnSkeletonJoint eJoint, History **history) {
	switch (eJoint)
	{
	case XN_SKEL_RIGHT_HAND:
		*history = &g_RightHandPositionHistory;
		break;

	case XN_SKEL_LEFT_HAND:
		*history = &g_LeftHandPositionHistory;
		break;

	default:
		*history = 0;
		return false;
	};

	return true;
}


void handtrajectory(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint, bool updateHistory)
{
	XnSkeletonJointPosition joint;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5){
           return;
        }
            
        XnPoint3D pt_world, pt_screen;
	pt_world = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, &pt_world, &pt_screen);
      
	History *history;
	if (GetHistoryForJoint (eJoint, &history) == false) return;

	if (updateHistory) history->StoreValue (pt_world, pt_screen); // store value in the history

	// Visualize history
	//
	XnFloat pVertexBuffer [HISTORY_DRAW_SIZE * sizeof (float) * 3];
	XnFloat *pVertex = pVertexBuffer;

	// Prepare vertex buffer for drawing
	XnPoint3D pt;
	for (int k = 0; k < history->Size(); ++k) {
		history->GetValueScreen (k, pt);

		*pVertex++ = pt.X;
		*pVertex++ = pt.Y;
		*pVertex++ = 0.0f;
	}

	glColor3f(0.f, 1.f, 0.f);
	glVertexPointer(3, GL_FLOAT, 0, pVertexBuffer);

	// draw trajectory
	glLineWidth(2);
	glDrawArrays(GL_LINE_STRIP, 0, history->Size());

	// draw history points
	glPointSize(8);
	glDrawArrays(GL_POINTS, 0, history->Size());
}
        
  void processKinect(KinectController &kinect_controller)
      {
        XnUserID users[3];
        XnUInt16 users_count = 3;
        xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        xn::DepthGenerator& depthGenerator = kinect_controller.getDepthGenerator();
        UserGenerator.GetUsers(users, users_count);
	skeleton_markers::Skeleton g_skel;
        for (int i = 0; i < users_count; ++i)
        {
           
          XnUserID user = users[i];
          if (!UserGenerator.GetSkeletonCap().IsTracking(user)){		
            continue;
           }
					XnPoint3D com;
				  	UserGenerator.GetCoM(user, com);
					depthGenerator.ConvertRealWorldToProjective(1, &com, &com);
				        glVertex2f(com.X, com.Y);			           
				      
				        // Calculate the distance between human and camera
				        float dist = com.Z /1000.0f;
					geometry_msgs::Twist vel;								

					if(g_LeftHandPositionHistory.Speed()>150)
						{
						 vel.linear.x = 0.00; 
						 vel.angular.z = -0.08;
						 ROS_INFO("Robot is moving anticlockwise %.2f", g_LeftHandPositionHistory.Speed());
						 
						}

					else if(g_RightHandPositionHistory.Speed()>150 && g_LeftHandPositionHistory.Speed()>150)
					  {
						 
						 ROS_INFO("Robot is moving....");
						 ros::Duration(1.0).sleep(); // sleep for 1 second
						 ROS_INFO("Robot.....");
						 
					    }	
					else if(g_RightHandPositionHistory.Speed()>150){
						
						 vel.linear.x = 0.00;
						 vel.angular.z = 0.08;
						 ROS_INFO("Robot is moving clockwise");
						 
					    }
					else{
				                //velocity command send to robot for follow human.
						if(dist >=1.5)
							{
								ROS_INFO("I am following you");
								vel.linear.x = 0.1;
								vel.angular.z = 0.0;
							}
						else if(dist <=1.5 && dist >=1.0)
							{
								ROS_INFO("I am rounding ...");
								vel.linear.x = 0.0;
								vel.angular.z = 0.1;
							}
						else{
							ROS_INFO(" You are too near to me");
							vel.linear.x = -0.1;
							vel.angular.z = 0.0;
						    }											 
					    } 
 
          handtrajectory(UserGenerator, depthGenerator, user, XN_SKEL_RIGHT_HAND, true); 
	  handtrajectory(UserGenerator, depthGenerator, user, XN_SKEL_LEFT_HAND, true);           
          publishTransform(kinect_controller, user, XN_SKEL_HEAD,           fixed_frame, "head", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_NECK,           fixed_frame, "neck", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_TORSO,          fixed_frame, "torso", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_SHOULDER,  fixed_frame, "left_shoulder", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_ELBOW,     fixed_frame, "left_elbow", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HAND,      fixed_frame, "left_hand", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_SHOULDER, fixed_frame, "right_shoulder", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_ELBOW,    fixed_frame, "right_elbow", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HAND,     fixed_frame, "right_hand", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HIP,       fixed_frame, "left_hip", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_KNEE,      fixed_frame, "left_knee", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_FOOT,      fixed_frame, "left_foot", g_skel);

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HIP,      fixed_frame, "right_hip", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_KNEE,     fixed_frame, "right_knee", g_skel);
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_FOOT,     fixed_frame, "right_foot", g_skel);
         
	                       /////
		// Input Joint Positions And Orientations from kinect 																													
		/////	
		
		
		// Upper Joints positions
		XnSkeletonJointPosition joint_position_head;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position_head);
		KDL::Vector head(joint_position_head.position.X, joint_position_head.position.Y, joint_position_head.position.Z);
                
	        
		XnSkeletonJointPosition joint_position_neck;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position_neck);
		KDL::Vector neck(joint_position_neck.position.X, joint_position_neck.position.Y, joint_position_neck.position.Z);
		
		XnSkeletonJointPosition joint_position_torso;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position_torso);
		KDL::Vector torso(joint_position_torso.position.X, joint_position_torso.position.Y, joint_position_torso.position.Z);
					
        
               		// Left Arm ****
		XnSkeletonJointPosition joint_position_left_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position_left_hand);
		KDL::Vector left_hand(joint_position_left_hand.position.X, joint_position_left_hand.position.Y, joint_position_left_hand.position.Z);
   

		XnSkeletonJointPosition joint_position_left_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position_left_elbow);
		KDL::Vector left_elbow(joint_position_left_elbow.position.X, joint_position_left_elbow.position.Y, joint_position_left_elbow.position.Z);        
    

		XnSkeletonJointPosition joint_position_left_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position_left_shoulder);
		KDL::Vector left_shoulder(joint_position_left_shoulder.position.X, joint_position_left_shoulder.position.Y, joint_position_left_shoulder.position.Z);        	        	
   

               		// Right Arm ****
		XnSkeletonJointPosition joint_position_right_hand;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position_right_hand);
		KDL::Vector right_hand(joint_position_right_hand.position.X, joint_position_right_hand.position.Y, joint_position_right_hand.position.Z);
    
		
		XnSkeletonJointPosition joint_position_right_elbow;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position_right_elbow);
		KDL::Vector right_elbow(joint_position_right_elbow.position.X, joint_position_right_elbow.position.Y, joint_position_right_elbow.position.Z);        
    
		
		XnSkeletonJointPosition joint_position_right_shoulder;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position_right_shoulder);
		KDL::Vector right_shoulder(joint_position_right_shoulder.position.X, joint_position_right_shoulder.position.Y, joint_position_right_shoulder.position.Z);
    
                
            // Right Leg ****
		XnSkeletonJointPosition joint_position_right_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, joint_position_right_hip);
		KDL::Vector right_hip(joint_position_right_hip.position.X, joint_position_right_hip.position.Y, joint_position_right_hip.position.Z);
   

		XnSkeletonJointPosition joint_position_right_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, joint_position_right_knee);
		KDL::Vector right_knee(joint_position_right_knee.position.X, joint_position_right_knee.position.Y, joint_position_right_knee.position.Z);
   
		XnSkeletonJointPosition joint_position_right_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, joint_position_right_foot);
		KDL::Vector right_foot(joint_position_right_foot.position.X, joint_position_right_foot.position.Y, joint_position_right_foot.position.Z);
  

               		// Left Leg ****
		XnSkeletonJointPosition joint_position_left_hip;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, joint_position_left_hip);
		KDL::Vector left_hip(joint_position_left_hip.position.X, joint_position_left_hip.position.Y, joint_position_left_hip.position.Z);
    
		XnSkeletonJointPosition joint_position_left_knee;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, joint_position_left_knee);
		KDL::Vector left_knee(joint_position_left_knee.position.X, joint_position_left_knee.position.Y, joint_position_left_knee.position.Z);
  
		XnSkeletonJointPosition joint_position_left_foot;
		UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, joint_position_left_foot);
		KDL::Vector left_foot(joint_position_left_foot.position.X, joint_position_left_foot.position.Y, joint_position_left_foot.position.Z);
  	
	
           // Upper joints rotations
              XnSkeletonJointOrientation joint_orientation_head;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation_head);
		XnFloat* h = joint_orientation_head.orientation.elements;
		KDL::Rotation head_rotation(h[0], h[1], h[2],
																h[3], h[4], h[5],
																h[6], h[7], h[8]);
	  double head_roll, head_pitch, head_yaw;
	  head_rotation.GetRPY(head_roll, head_pitch, head_yaw);
        
           // head yaw
	  static double head_angle_yaw = 0;
          if (joint_orientation_head.fConfidence >= 0.5)
          {     
	    head_angle_yaw = head_pitch;
          }

          // head pitch
	  static double head_angle_pitch = 0;
          if (joint_orientation_head.fConfidence >= 0.5)
          {           
	    head_angle_pitch = head_roll;
          }
		
	  XnSkeletonJointOrientation joint_orientation_neck;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_NECK, joint_orientation_neck);
		XnFloat* n = joint_orientation_neck.orientation.elements;
	  KDL::Rotation neck_rotation(n[0], n[1], n[2],
																n[3], n[4], n[5],
																n[6], n[7], n[8]);
		double neck_roll, neck_pitch, neck_yaw;
	  neck_rotation.GetRPY(neck_roll, neck_pitch, neck_yaw);	
           
         XnSkeletonJointOrientation joint_orientation_torso;
		UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation_torso);
	  XnFloat* t = joint_orientation_torso.orientation.elements;
	  KDL::Rotation torso_rotation(t[0], t[1], t[2],
																 t[3], t[4], t[5],
																 t[6], t[7], t[8]);
		double torso_roll, torso_pitch, torso_yaw;
	  torso_rotation.GetRPY(torso_roll, torso_pitch, torso_yaw);	
                   // torso yaw
	  static double torso_angle_yaw = 0;
          if (joint_orientation_torso.fConfidence >= 0.5)
          {           
          	torso_angle_yaw = torso_pitch;
          }           
   

     // the kinect and robot are in different rotation spaces,

    /////
		// Process and output joint rotations to the  robot.																														
		/////
    
    // ARMS for robot
		
  
		// left elbow roll ****
		KDL::Vector left_elbow_hand(left_hand - left_elbow);
		KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
		left_elbow_hand.Normalize();
		left_elbow_shoulder.Normalize();
            	static double left_elbow_angle_roll = 0;
		if (joint_position_left_hand.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
			left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
			left_elbow_angle_roll = left_elbow_angle_roll - PI;
                       // cout << left_elbow_angle_roll<<endl;
		}
		
		// right elbow roll ****
		KDL::Vector right_elbow_hand(right_hand - right_elbow);
		KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);
		right_elbow_hand.Normalize();
		right_elbow_shoulder.Normalize();
		static double right_elbow_angle_roll = 0;
		if (joint_position_right_hand.fConfidence >= 0.5 && 
				joint_position_right_elbow.fConfidence >= 0.5 && 
				joint_position_right_shoulder.fConfidence >= 0.5)
		{          
			right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
			right_elbow_angle_roll = -(right_elbow_angle_roll - PI);
		} 
           	// left shoulder roll ****
		KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
		KDL::Vector left_shoulder_right_shoulder(right_shoulder - left_shoulder);
		left_shoulder_elbow.Normalize();
		left_shoulder_right_shoulder.Normalize();
   		static double left_shoulder_angle_roll = 0;
		if (joint_position_right_shoulder.fConfidence >= 0.5 && 
				joint_position_left_elbow.fConfidence >= 0.5 && 
				joint_position_left_shoulder.fConfidence >= 0.5)
		{
			left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_right_shoulder));
			left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
		}
						 
		// right shoulder roll ****
		KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
		KDL::Vector right_shoulder_left_shoulder(left_shoulder - right_shoulder);
		right_shoulder_elbow.Normalize();
		right_shoulder_left_shoulder.Normalize();
		static double right_shoulder_angle_roll = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5 && 
				joint_position_right_elbow.fConfidence >= 0.5 && 
				joint_position_right_shoulder.fConfidence >= 0.5)
		{     
			right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_left_shoulder));
			right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
		

               }
      
             // left shoulder pitch ****
		static double left_shoulder_angle_pitch = 0;
		if (joint_position_left_shoulder.fConfidence >= 0.5 &&
        joint_position_left_elbow.fConfidence >= 0.5)
		{ 
		  left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
			left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
		  
    }
	  
	
		// right shoulder pitch ****
		static double right_shoulder_angle_pitch = 0;
	  if (joint_position_right_shoulder.fConfidence >= 0.5)
		{
			right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
			right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
			
  	}
         
		// left shoulder yaw ****
		static double left_shoulder_angle_yaw = 0;
		
		if (joint_position_left_shoulder.fConfidence >= 0.5)
    {
      
        left_shoulder_angle_yaw = asin(left_elbow_hand.x()); 
        //left_shoulder_angle_yaw = left_shoulder_roll;
    }

		// right shoulder yaw ****
		static double right_shoulder_angle_yaw = 0;
		
    if (joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_yaw = asin(right_elbow_hand.x());
      right_shoulder_angle_yaw = -(right_shoulder_angle_yaw);
    }
		
    // Use head for counter balancing
    static double hp_min = -0.1;
    static double hp_max = 0.65;
    // if this number is close to zero head should be hp_min
    // if it is close to or above HALFPI then head should be hp_max
		double arm_pitch_factor = (fabs(right_shoulder_angle_pitch) + 
                               fabs(left_shoulder_angle_pitch)) / 2.0;
    if (arm_pitch_factor >= HALFPI)
      head_angle_pitch = 0.65;
    else
      head_angle_pitch = hp_min + ((arm_pitch_factor / HALFPI) * (hp_max - hp_min));
   
    // head follows arm most sideways
  
    double arm_roll_factor = (right_shoulder_angle_roll + 
                             left_shoulder_angle_roll) / 2.0;
    head_angle_yaw = arm_roll_factor * -1.1;
    
         
      sensor_msgs::JointState js; 
    
      js.name.push_back("elbow_left_roll");
      js.position.push_back(left_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_roll");
      js.position.push_back(left_shoulder_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_pitch");
      js.position.push_back(left_shoulder_angle_pitch);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_left_yaw");
      js.position.push_back(left_shoulder_angle_yaw);
      js.velocity.push_back(10);
  
      js.name.push_back("elbow_right_roll");
      js.position.push_back(right_elbow_angle_roll);
      js.velocity.push_back(10);
      js.name.push_back("shoulder_right_roll");
      js.position.push_back(right_shoulder_angle_roll);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_right_pitch");
      js.position.push_back(right_shoulder_angle_pitch);
      js.velocity.push_back(10);          
      js.name.push_back("shoulder_right_yaw");
      js.position.push_back(right_shoulder_angle_yaw);
      js.velocity.push_back(10);       
   
      js.name.push_back("neck_yaw");
      js.position.push_back(head_angle_yaw);
      js.velocity.push_back(10);          
      js.name.push_back("neck_pitch");
      js.position.push_back(head_angle_pitch);
      js.velocity.push_back(10);    
   		
      joint_states_pub_.publish(js);
      
      /////
    //  Following is IK method
    /////

    KDL::Vector left_hand_torso  = left_hand;  // - torso;
    KDL::Vector right_hand_torso = right_hand; // - torso;
    
    double scale = 0.0003432;

    left_hand_torso  = scale * left_hand_torso;
    right_hand_torso = scale * right_hand_torso;

    geometry_msgs::Point p;

    if (joint_position_left_hand.fConfidence >= 0.5)
    {
      p.x = -left_hand_torso.z();
      p.y = -left_hand_torso.x();
      p.z = left_hand_torso.y();
      left_arm_destination_pub_.publish(p);
    }

    if (joint_position_right_hand.fConfidence >= 0.5)
    {
      p.x = -right_hand_torso.z();
      p.y = -right_hand_torso.x();
      p.z = right_hand_torso.y();
      right_arm_destination_pub_.publish(p);
    }

    // The robot will receive and try to mirror my position
    geometry_msgs::Point torso_destination;
    torso_destination.x = -torso.z() / 1000.0;
    torso_destination.y = -torso.x() / 1000.0;
    //robot_destination.z = torso.y() / 1000.0;
    torso_destination.z = torso_pitch;
    torso_destination_pub_.publish(torso_destination);



              g_skel.user_id = user;
	      g_skel.header.stamp = ros::Time::now();
	      g_skel.header.frame_id = fixed_frame;
              skeleton_pub_.publish(g_skel);
	      cmdVelPub.publish(vel);
      break;	// only read first user 
      }
}
         
  private:
    void enableJointGroupCB(const skeleton_markers::EnableJointGroupConstPtr& msg);
    ros::Publisher  skeleton_pub_;    
    ros::Publisher  torso_destination_pub_;
    ros::Publisher  left_arm_destination_pub_;
    ros::Publisher  right_arm_destination_pub_;
    ros::Publisher  cmdVelPub;
    ros::Publisher  joint_states_pub_;  
    ros::Subscriber  enable_joint_group_sub_;
       
  };


void SkeletonTracker::enableJointGroupCB(const skeleton_markers::EnableJointGroupConstPtr& msg)
{
	for (size_t i=0; i < msg->jointGroups.size(); i++)
  {
  	if (msg->jointGroups[i] == "legs")
		{
			legs_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "arms")
		{
			right_arm_enabled_ = left_arm_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "left_arm")
		{
		  left_arm_enabled_ = msg->enabledStates[i];
		}	
		else if (msg->jointGroups[i] == "right_arm")
		{
			right_arm_enabled_ = msg->enabledStates[i];
		}
		else if (msg->jointGroups[i] == "motions")
		{
			motion_enabled_ = msg->enabledStates[i];      
  	}
	}
}

}


#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
KinectController g_kinect_controller;
skeleton_tracker::SkeletonTracker g_skeleton_tracker;

int depth_window, rgb_window;

void glutIdle (void)
{
  	g_kinect_controller.getContext().WaitAndUpdateAll();
  	g_skeleton_tracker.processKinect(g_kinect_controller);

	glutSetWindow(depth_window);
	glutPostRedisplay();

	glutSetWindow(rgb_window);
	glutPostRedisplay();
}

void glutDepthDisplay (void)
{
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
  	g_kinect_controller.getDepthGenerator().GetMetaData(depthMD);
  	g_kinect_controller.getUserGenerator().GetUserPixels(0, sceneMD);  
	 
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
  
        glDisable(GL_TEXTURE_2D);
  
        kinect_display_drawDepthMapGL(depthMD, sceneMD);
	kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),
                                g_kinect_controller.getDepthGenerator(), true);  
         
	glutSwapBuffers();
}

void glutRgbDisplay (void)
{
	xn::ImageMetaData imageMD;
  	g_kinect_controller.getImageGenerator().GetMetaData(imageMD);
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0, imageMD.XRes(), imageMD.YRes(), 0, -1.0, 1.0);
  
        glDisable(GL_TEXTURE_2D);
  
        kinect_display_drawRgbMapGL(imageMD);
	kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),
                                g_kinect_controller.getDepthGenerator(), false); 
   	  
	glutSwapBuffers();
	
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27: exit(1);
                 break;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skeleton_tracker");
  ros::NodeHandle n;
  ros::NodeHandle np("~");
  string filepath;
  bool is_a_recording;
  np.getParam("load_filepath", filepath); 
  np.param<bool>("load_recording", is_a_recording, false);   
  
  g_skeleton_tracker.init();
  g_kinect_controller.init(filepath.c_str(), is_a_recording);

  glutInit(&argc, argv);
 
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  
  depth_window = glutCreateWindow ("Skeleton Depth");
  glutDisplayFunc(glutDepthDisplay);

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY); 

  glutKeyboardFunc(glutKeyboard);

  rgb_window = glutCreateWindow ("Skeleton RGB");
  glutDisplayFunc(glutRgbDisplay);
  
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);  
  
  glutIdleFunc(glutIdle);
  glutMainLoop();
  
  g_kinect_controller.shutdown();
  
  return 0;
}


