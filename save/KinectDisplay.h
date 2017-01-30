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

void glPrintString(void *font, char *str);

void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd,
                                   const xn::SceneMetaData& smd);

void kinect_display_drawRgbMapGL(const xn::ImageMetaData& imd, AttentionMap &attention_map);//, ObjectsFinderBase &objectsFinder);
                                   
void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                 xn::DepthGenerator& depthGenerator, bool isDepthPass);

void DrawCircle(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint,
		float radius, XnFloat *color3f); 

void DrawBezierCurve(const std::vector<XnPoint3D> &controlPoints, int numPoints = 16);

                                  
#endif
