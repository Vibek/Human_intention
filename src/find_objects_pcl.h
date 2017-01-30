#ifndef __OBJECTS_FINDER_PCL_H__
#define __OBJECTS_FINDER_PCL_H__

#include "ros/ros.h"
//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <XnV3DVector.h>
#include <XnCppWrapper.h>
#include <XnVPointControl.h>

#include <sensor_msgs/PointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/openni_grabber.h>

//#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include "find_objects_base.h"



class  ObjectFinderPCL: public ObjectsFinderBase
{

private:
	int cols, rows;

	cv::Mat src1;
	cv::Mat depth;
	pcl::PointCloud<pcl::PointXYZ> tmp_pc;

	cv::Mat depth_initial, depth_subtracted,mask1,mask,mask2,mask3,hsv1,range;
	int initial_count;
	pcl::PointCloud<pcl::PointXYZ> pc;

public:
	virtual void init(int width, int height);
	virtual cv::Mat detectObjects(cv::Mat frame);
	virtual void detectObjectsNI(const sensor_msgs::ImageConstPtr& original_image) {};

	virtual void processDepth(const XnDepthPixel* pDepth);
	virtual void processDepthNI(const sensor_msgs::ImageConstPtr& depth_image) {};

	void processPoint(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};

#endif
