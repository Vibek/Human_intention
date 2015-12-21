#ifndef KINECT_DISPLAY_
#define KINECT_DISPLAY_

#include <map>
#include <list>
#include <XnCppWrapper.h>
#include <XnVPointControl.h>




void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd,
                                   const xn::SceneMetaData& smd);
                                   
void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                 xn::DepthGenerator& depthGenerator);

                                  
#endif
