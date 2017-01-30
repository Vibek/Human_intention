/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef KINECT_DISPLAY_
#define KINECT_DISPLAY_

#include <ros/ros.h>
#include <ros/console.h>

#include <map>
#include <list>
#include <vector>

#include <XnCppWrapper.h>
#include <XnVPointControl.h>

#include "attention_map.hpp"
#include "find_objects.h"
#include "OutputData.h"

void glPrintString(void *font, char *str);

void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd,
                                   const xn::SceneMetaData& smd);

void kinect_display_drawRgbMapGL(const xn::ImageMetaData& imd, AttentionMap &attention_map);//, ObjectsFinderBase &objectsFinder);
                                   
void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                 xn::DepthGenerator& depthGenerator, bool isDepthPass);

void DrawCircle(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint,
		float radius, const XnFloat *color3f); 

void DrawCircleOnScreen(XnPoint3D screenPt, float radius, const XnFloat *color3f); 

void DrawTransparentCircleOnScreen(XnPoint3D screenPt, float radius, const XnFloat *color4f);

void DrawRectangleOnScreen(XnPoint3D screenPt, float width, float height, const XnFloat *color3f); 

void DrawTransparentRectangleOnScreen(XnPoint3D screenPt, float width, float height, const XnFloat *color4f);

static const XnFloat transp_colors[][4] = {
	{0, 0, 0, 0.5f},			// black
	{0.5f, 0.5f, 0.5f, 0.5f}, //gray
	{1, 1, 0, 0.5f},			// yellow
	{0, 0, 1, 0.5f},			// blue
	{1, 0, 0, 0.5f},			// red
	{1, 1, 1, 0.5f}			// white
};


static const XnFloat traj_colors[][3] = {
	{0, 0, 0},			// black
	{0.5f, 0.5f, 0.5f}, //gray
	{1, 1, 0},			// yellow
	{0, 0, 1},			// blue
	{1, 0, 0},			// red
	{1, 1, 1}			// white
};
void DrawBezierCurve(const std::vector<XnPoint3D> &controlPoints, const XnFloat *color3f, int lineWidth = 1, int numPoints = 16);

                                  
const int NumObjectsOfInterest = 2;
XnPoint3D GetObjectOfInterestProjPos(int k);
XnPoint3D GetObjectOfInterestWorldPos(int k);

void SetObjectOfInterestSelected(int objectIndex, bool flag);

#endif
