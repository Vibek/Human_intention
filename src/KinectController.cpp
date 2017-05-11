/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

//OpenNI libraries
#include <XnOpenNI.h>
#include <XnCodecIDs.h>

#include "KinectDisplay.h"
#include "KinectController.h"

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"


#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

#define CHECK_ERRORS(nRetVal, errors, what)		\
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)	\
	{										\
		XnChar strError[1024];				\
		errors.ToString(strError, 1024);	\
		printf("%s\n", strError);			\
		return (nRetVal);						\
	}

xn::Context        KinectController::g_Context;
xn::DepthGenerator KinectController::g_DepthGenerator;
xn::UserGenerator  KinectController::g_UserGenerator;
xn::ImageGenerator KinectController::g_ImageGenerator;

XnBool KinectController::g_bNeedPose = FALSE;
XnChar KinectController::g_strPose[20] = "";

XnBool KinectController::g_bhascal = FALSE;

KinectController::KinectController()
{
}

// Accessors
xn::UserGenerator& KinectController::getUserGenerator()
{
	return g_UserGenerator;
}

xn::DepthGenerator& KinectController::getDepthGenerator()
{
	return g_DepthGenerator;
}

xn::ImageGenerator& KinectController::getImageGenerator()
{
	return g_ImageGenerator;
}

xn::Context& KinectController::getContext()
{
	return g_Context;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
	
     //if (KinectController::g_bhascal)
	//{
		// Calibration succeeded
		KinectController::g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, XN_CALIBRATION_FILE_NAME);
		KinectController::g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);		
		KinectController::g_UserGenerator.GetSkeletonCap().StartTracking(nId);		
		printf("Calibration data has been Loaded!\n");
		
		
	//}
// Detected first user: request calibration pose detection
     //else
	//{		// New user found
	  if (KinectController::g_bNeedPose)
	   {
		
		KinectController::g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(KinectController::g_strPose, nId);
				
	   }

	  else
	    {
		KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	    } 
	//}       
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose %s detected for user %d\n", strPose, nId);
	
// If we already calibrated on a user, just load that calibration
   // if (KinectController::g_bhascal)
	//{
		// Calibration succeeded
		KinectController::g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, XN_CALIBRATION_FILE_NAME);
		KinectController::g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);	
		KinectController::g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		printf("Calibration data has been Loaded!\n");
		
		
	//}
    // else
      //  {
	    KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	//}
}

// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	printf("Calibration started for user %d\n", nId);
}

// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	
	if (bSuccess)
	{
		// Save the user's calibration
		//KinectController::g_bhascal = true;		
		printf("Calibration data has been saved!\n");		
		KinectController::g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		KinectController::g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(nId, XN_CALIBRATION_FILE_NAME);		
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		printf("Saving calibration data has failed!\n");
		if (KinectController::g_bNeedPose)
		{
			KinectController::g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(KinectController::g_strPose, nId);
		}
		else
		{
			KinectController::g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

// callback for session start
void XN_CALLBACK_TYPE SessionStarting(const XnPoint3D& ptPosition, void* UserCxt)
{
	printf("Session start: (%f,%f,%f)\n", ptPosition.X, ptPosition.Y, ptPosition.Z);
	//g_SessionState = IN_SESSION;
}

int KinectController::init(const char* path, bool recording)
{
        XnStatus nRetVal = XN_STATUS_OK;
        xn::EnumerationErrors	errors;
	if (recording)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(path);
               if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	           {
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return 1;
	           } 
               
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", path, xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{
		nRetVal = g_Context.InitFromXmlFile(path);
		CHECK_RC(nRetVal, "InitFromXml");
	}

       	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	//nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_HandsGenerator);
        //CHECK_RC(nRetVal, "Find hand generator");     

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_ImageGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find image generator.... HELP!!! ");
	} 


	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
  
        return 0;
}

int KinectController::shutdown()
{
	g_Context.Shutdown();
  return 0;
}


/****** Improtant functions which can use for various activities *****/
/*

		virtual XnStatus SaveCalibrationDataToFile(XnUserID user, const XnChar* strFileName) = 0;
		virtual XnStatus LoadCalibrationDataFromFile(XnUserID user, const XnChar* strFileName) = 0;
		virtual XnStatus SaveCalibrationData(XnUserID user, XnUInt32 nSlot) = 0;
		virtual XnStatus LoadCalibrationData(XnUserID user, XnUInt32 nSlot) = 0;
		virtual XnStatus ClearCalibrationData(XnUInt32 nSlot) = 0;
		virtual XnBool IsCalibrationData(XnUInt32 nSlot) = 0;
		virtual XnStatus StartTracking(XnUserID user) = 0;
		virtual XnStatus StopTracking(XnUserID user) = 0;
*/
