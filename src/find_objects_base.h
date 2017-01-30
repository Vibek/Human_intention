#ifndef __OBJECTS_FINDER_BASE_H__
#define __OBJECTS_FINDER_BASE_H__

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

class ObjectsFinderBase
{
public:
	virtual void init(int width, int height) = 0;
	virtual cv::Mat detectObjects(cv::Mat frame) { return frame; };
	virtual void detectObjectsNI(const sensor_msgs::ImageConstPtr& original_image) {};

	virtual void processDepth(const XnDepthPixel* pDepth) {};
	virtual void processDepthNI(const sensor_msgs::ImageConstPtr& depth_image) {};
};


#endif
