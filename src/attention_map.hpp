/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef __ATTENTION_MAP_HPP__
#define __ATTENTION_MAP_HPP__

#include <XnCppWrapper.h>
#include <XnVPointControl.h>

#include <opencv2/opencv.hpp>
using namespace cv;


struct AttentionMap
{
	void init(int fps, int width, int height);

	void overlay(unsigned char* pDestImage, int imageWidth, int imageHeight);

private:
	void create_kernel();

	void update();
	void CollectHeatPoints(XnSkeletonJoint eJoint, std::vector<XnPoint3D> &heat_points);
	void heat_point(int x, int y);

	void overlay_heatmap(Mat frame);

	void fade();

	Mat m_heatmap;	
	Mat m_kernel;
	Mat m_ones;
	Mat m_zeros;
	Mat m_fade_mat;

	int m_last_update_time;
};

#endif // __ATTENTION_MAP_HPP__
