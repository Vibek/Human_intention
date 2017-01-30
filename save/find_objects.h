/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef FIND_OBJECT_H
#define FIND_OBJECT_H

//OpenCV libraries
#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//C++ libraries
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "find_objects_base.h"

//using namespace cv;
using namespace std;


class ObjectsFinderFeature2D : public ObjectsFinderBase
{
public:
	void init(int width, int height);

protected:
	virtual bool detectAndCompute(cv::Mat &img, vector<cv::KeyPoint> &kp, cv::Mat &desc) = 0;

	string removePath(string filename);
	string removeExtension(string filename);
	bool readObjectsFile(char *filename);

	vector<cv::Mat> img_objects;
	vector <string> object_names;

	vector< vector<cv::KeyPoint> > keypoints_objects;
	vector<cv::Mat> descriptors_objects;

	vector<cv::KeyPoint> keypoints_scene;
	cv::Mat descriptors_scene;

	// image size 
	cv::Size scene_size;

	int current_object;
	bool current_only;
};

#if 1
class ObjectsFinderSURF : public ObjectsFinderFeature2D {
public:
	void init(int width, int height);

	cv::Mat detectObjects(cv::Mat frame);

protected:
	bool detectAndCompute(cv::Mat &img, vector<cv::KeyPoint> &kp, cv::Mat &desc);

private:

	//internal vars
	
	cv::SurfFeatureDetector *detector;
	cv::SurfDescriptorExtractor extractor;
	cv::FlannBasedMatcher matcher;

	bool isSquared( vector<cv::Point> points);
	bool readGraphFile(char *filename);

	int minHessian; 

	//graph: id0 -> id1 id2 ed4 ..
	vector < vector <int> > visibility_graph;
};
#endif


class ObjectsFinderORB : public ObjectsFinderFeature2D {
public:
	void init(int width, int height);

	cv::Mat detectObjects(cv::Mat frame);

protected:
	bool detectAndCompute(cv::Mat &img, vector<cv::KeyPoint> &kp, cv::Mat &desc);

private:
		//internal vars
	cv::Ptr<cv::ORB> orbDetector;
	cv::Ptr<cv::DescriptorMatcher> matcher;


};

#if 1
class ObjectsFinderCMB : public ObjectsFinderFeature2D {
public:
	void init(int width, int height);

	cv::Mat detectObjects(cv::Mat frame);

protected:
	bool detectAndCompute(cv::Mat &img, vector<cv::KeyPoint> &kp, cv::Mat &desc);

private:
		//internal vars
	cv::Ptr<cv::FeatureDetector> CMBdetector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	cv::Ptr<cv::DescriptorExtractor> CMBextractor; 


};


#endif

#endif //FIND_OBJECT_H
