#include "find_objects_color.h"

#include "ros/ros.h"
#//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image Deping
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <cmath>
#include <tf/transform_listener.h>




//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
cv_bridge::CvImagePtr cv_Depth;
cv_bridge::CvImagePtr cv_ptr;

ObjectsFinderColor *g_pObjectsFinder;
   
// ObjectsFinderColor
//
void ObjectsFinderColor::init(int width, int height)
{
	Object::loadObjectTypes();

	objectsToFind.push_back(Object(Object::RED));
	objectsToFind.push_back(Object(Object::GREEN));
	objectsToFind.push_back(Object(Object::BLUE));
	objectsToFind.push_back(Object(Object::YELLOW));
	objectsToFind.push_back(Object(Object::BLACK));
}

void ObjectsFinderColor::detectObjectsNI(const sensor_msgs::ImageConstPtr& original_image)
{
     cv_ptr = cv_bridge::toCvCopy(original_image, "mono8");
     cv::Mat frame;
     frame = cv_ptr->image;
     cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
        cv::imshow("RGB", frame);
	cv::waitKey(0); //wait infinite time for a keypress
	cv::destroyWindow("RGB");

	for (int obj = 0; obj < (int)objectsToFind.size(); ++obj)
	{
		Object& objectToFind = objectsToFind[obj];

		cv::cvtColor(frame, HSV, cv::COLOR_BGR2HSV);
		cv::inRange(HSV, objectToFind.HSVmin, objectToFind.HSVmax, threshold);
		
		morphOps(threshold);
		cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
		cv::imshow("view",threshold);
		cv::waitKey(0);
		cv::destroyWindow("view");
		trackFilteredObject(objectToFind, threshold, HSV, frame);
		cv::namedWindow("Qrcode", cv::WINDOW_AUTOSIZE);
		cv::imshow("Qrcode", HSV);
		cv::waitKey(0);
		cv::destroyWindow("Qrcode");
                //cv::namedWindow("Rotation Image");
		//cv::imshow("Rotataion Image", objectToFind);
		
	}
        
	//CreatePublisherObject();
	SceneObjects.clear();
          //wait infinite time for a keypress
	
	//return frame;
}


//void ObjectsFinderColor::processDepth(const XnDepthPixel* pDepth)

void ObjectsFinderColor::processDepthNI(const sensor_msgs::ImageConstPtr& depth_image)
{
	int sizes[2] = {480, 640};
	cv_Depth = cv_bridge::toCvCopy(depth_image, "16UC1");
	cv::Mat frame;
        frame = cv_ptr->image;
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
        cv::imshow("depth", frame);
	cv::waitKey(0); //wait infinite time for a keypress
	cv::destroyWindow("depth");

}

std::string intToString(int number){

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void ObjectsFinderColor::drawObject(std::vector<Object> theObjects, cv::Mat &frame, cv::Mat &temp, std::vector< std::vector<cv::Point> > contours, std::vector<cv::Vec4i> hierarchy){

	//MyObjects.push_back(theObjects);
	

	for(int i = 0; i<theObjects.size(); i++)
	{
		//Actualize object of scene
		//Resolution Kinect 640 x 480
		// A RAJOUTER
		cv::drawContours(frame,contours,i,theObjects.at(i).Color,3,8,hierarchy);
		// Fais le rond du centre de l'objet
		cv::circle(frame, cv::Point(theObjects.at(i).XPos,theObjects.at(i).YPos),5,theObjects.at(i).Color ,1);
		
		cv::putText(frame,intToString(theObjects.at(i).XPos)+ " , " + intToString(theObjects.at(i).YPos),cv::Point(theObjects.at(i).XPos, theObjects.at(i).YPos - 20), 1, 1, theObjects.at(i).Color);
		
		break;
	}
}

void ObjectsFinderColor::morphOps(cv::Mat thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	cv::Mat erodeElement = getStructuringElement( cv::MORPH_RECT, cv::Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = getStructuringElement( cv::MORPH_RECT, cv::Size(8,8));

	cv::erode(thresh,thresh,erodeElement);
	cv::erode(thresh,thresh,erodeElement);

	cv::dilate(thresh,thresh,dilateElement);
	cv::dilate(thresh,thresh,dilateElement);
}

void ObjectsFinderColor::trackFilteredObject(Object theObject, cv::Mat &threshold, cv::Mat &HSV, cv::Mat &cameraFeed)
{
	std::vector <Object> objects;
	cv::Mat temp;
	threshold.copyTo(temp);
	
	//these two vectors needed for output of findContours
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	cv::findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){

			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				cv::Moments moment = cv::moments((cv::Mat)contours[index]);
				double area = moment.m00;

		//if the area is less than 20 px by 20px then it is probably just noise
		//if the area is the same as the 3/2 of the image size, probably just a bad filter
		//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object object;
					
					object.XPos = moment.m10/area;
					object.YPos = moment.m01/area;
					object.type = theObject.type;
					object.Color = theObject.Color;

					objects.push_back(object);
					//Je met mon objet trouve dans la liste des objets de la scene
					SceneObjects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				//ROS_INFO("%d",numObjects);
				drawObject(objects,cameraFeed,temp,contours,hierarchy);
				
				

				
			}

		}
		//else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",cv::Point(0,50),1,2,cv::Scalar(0,0,255),2);
	}
}

void ObjectsFinderColor::CreatePublisherObject()
{
	double depthObject;
	double depth0;
	double VecteurNew;
	int focalLengthY = 500; //515
	int focalLengthZ = 500;//220 - 400;

	for(int i = 0; i<SceneObjects.size(); i++)
	{
		depthObject = cv_Depth->image.at<short int>(cv::Point(SceneObjects.at(i).XPos,SceneObjects.at(i).YPos));
		
		depth0 = cv_Depth->image.at<short int>(cv::Point(320,240));
		depth0 = (double)depth0/1000;
		
		//Resolution Kinect 640 x 480
		double VecteurX = (double)depthObject/1000;
		double VecteurY = ((double)SceneObjects.at(i).XPos-320)/-1000;
		double VecteurZ = ((double)SceneObjects.at(i).YPos-240)/-1000;
/*

	    tf::Vector3 pointKinect(VecteurX,VecteurY,VecteurZ);
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try{
			listener.waitForTransform("/pos_color","/depth_frame",ros::Time(0), ros::Duration(2.0));
			listener.lookupTransform( "/pos_color","/depth_frame",ros::Time(0), transform);
		}
		 catch (tf::TransformException ex){
		 	ROS_ERROR("%s",ex.what());
		 	
		}
	    	tf::Vector3 pointOdom = transform * pointKinect;*/

	}
}




//ObjectsFinderColor::Object
// 
cv::Scalar ObjectsFinderColor::Object::g_HSVmin[Object::NUM_COLORS];
cv::Scalar ObjectsFinderColor::Object::g_HSVmax[Object::NUM_COLORS];
cv::Scalar ObjectsFinderColor::Object::g_Color[Object::NUM_COLORS];

void ObjectsFinderColor::Object::loadObjectTypes()
{
	g_HSVmin[RED] = cv::Scalar(144,93,129);
	g_HSVmax[RED] = cv::Scalar(186,255,255);
	g_Color[RED] = cv::Scalar(6,3,106);

	g_HSVmin[GREEN] = cv::Scalar(42,0,0);
	g_HSVmax[GREEN] = cv::Scalar(104,255,255);
	g_Color[GREEN] = cv::Scalar(32,75,49);

	g_HSVmin[BLUE] = cv::Scalar(90,115,76);
	g_HSVmax[BLUE] = cv::Scalar(130,255,255);
	g_Color[BLUE] = cv::Scalar(135,66,55);

	g_HSVmin[YELLOW] = cv::Scalar(20,176,177);
	g_HSVmax[YELLOW] = cv::Scalar(42,255,220);
	g_Color[YELLOW] = cv::Scalar(37,150,199);

	g_HSVmin[BLACK] = cv::Scalar(20,124,123);
	g_HSVmax[BLACK] = cv::Scalar(30,255,255);
	g_Color[BLACK] = cv::Scalar(0,0,0);
}


ObjectsFinderColor::Object::Object()
{
	type = BLUE;
	XPos = YPos = 0.0f;
	HSVmin = g_HSVmin[type];
	HSVmax = g_HSVmax[type];
	Color = g_Color[type];
}

ObjectsFinderColor::Object::Object(int type)
{
	this->type = type;
	XPos = YPos = 0.0f;
	HSVmin = g_HSVmin[type];
	HSVmax = g_HSVmax[type];
	Color = g_Color[type];
}


int main(int argc, char* argv[])
{
	//if we would like to calibrate our filter values, set to true.

	g_pObjectsFinder = new ObjectsFinderColor();


	ros::init(argc, argv, "object");
	ros::NodeHandle n;
	
	image_transport::ImageTransport depth_it(n);
	image_transport::Subscriber SubDepth = depth_it.subscribe("/camera/depth_registered/image_raw",1, &ObjectsFinderColor::processDepthNI, g_pObjectsFinder);

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw",1, &ObjectsFinderColor::detectObjectsNI, g_pObjectsFinder);

	
    ros::spin();
	return 0;
}
