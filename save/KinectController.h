/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef KINECT_CONTROLLER_H_
#define KINECT_CONTROLLER_H_

#include <XnCppWrapper.h>
#include <XnCyclicStackT.h>
#include <XnHashT.h>

// Hand position history length (positions)
#define MAX_HAND_TRAIL_LENGTH	10

class KinectController
{
  public:
    KinectController();
    
    int init(const char* path, bool recording=false);
    int shutdown();
    
    xn::UserGenerator&  getUserGenerator();
    xn::DepthGenerator& getDepthGenerator();
    xn::ImageGenerator& getImageGenerator();
    xn::Context&        getContext();
    
  public:
		static xn::Context g_Context;
		static xn::DepthGenerator g_DepthGenerator;
		static xn::UserGenerator g_UserGenerator;
		static xn::ImageGenerator g_ImageGenerator;
                static xn::HandsGenerator g_HandsGenerator;
		static XnBool g_bNeedPose;
		static XnChar g_strPose[20];
                
};

#endif 
