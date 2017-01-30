#ifndef __OBJECTS_FINDER_COLOR_H__
#define __OBJECTS_FINDER_COLOR_H__

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
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "find_objects_base.h"

class ObjectsFinderColor //: public ObjectsFinderBase
{
public:
	void init(int width, int height);

	void detectObjectsNI(const sensor_msgs::ImageConstPtr& original_image);

	void processDepthNI(const sensor_msgs::ImageConstPtr& depth_image);

public:
	struct Object
	{
		enum
		{
			RED=0, GREEN, BLUE, YELLOW, BLACK,

			NUM_COLORS
		};
	
		int XPos, YPos;
		int type;
		cv::Scalar HSVmin, HSVmax;
		cv::Scalar Color;

		Object();
		Object(int type);

		static cv::Scalar g_HSVmin[Object::NUM_COLORS];
		static cv::Scalar g_HSVmax[Object::NUM_COLORS];
		static cv::Scalar g_Color[Object::NUM_COLORS];

		static void loadObjectTypes();
	};

private:
	void morphOps(cv::Mat thresh);
	void trackFilteredObject(Object object, cv::Mat &threshold, cv::Mat &HSV, cv::Mat &cameraFeed);

	void drawObject(std::vector<Object> theObjects, cv::Mat &frame, cv::Mat &temp, std::vector< std::vector<cv::Point> > contours, std::vector<cv::Vec4i> hierarchy);

	void CreatePublisherObject();

private:
	std::vector<Object> objectsToFind;
	
	std::vector<Object> SceneObjects;	

	cv::Mat HSV, threshold;
	cv::Mat depthBuffer;
};

#endif

