/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

//ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

//OpenGL and OpenNI libraries
#include <GL/glut.h>
#include <XnV3DVector.h>

//C++ libraries
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "KinectDisplay.h"
#include "trajectory.h"

#define PI              3.14159265359
#define fps             30
#define MAX_DEPTH	10000
#define MAX_RGB		10000

XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;
XnBool g_bPrintFrameID = FALSE;

extern xn::ImageGenerator ImageGenerator;

ros::Publisher  cmdVelPub;

using std::string;
using namespace std;

float f = 525.0, D = 45.0;   

float g_pDepthHist[MAX_DEPTH];
float g_pRgbHist[MAX_RGB];

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}

GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

GLfloat texcoords[8];

void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	GLfloat verts[8] = {	topLeftX, topLeftY,
		topLeftX, bottomRightY,
		bottomRightX, bottomRightY,
		bottomRightX, topLeftY
	};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
	glFlush();
}

void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

	DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

XnFloat Colors[][3] =
{
	{0,1,1},
	{0,0,1}, // blue
	{0,1,0},
	{1,1,0}, // yellow
	{1,0,0}, // red
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{1,1,1},
        {0.5,0.5,0.5},	// Grey	
	{1,0,1},	// Purple
	
         
};
XnUInt32 nColors = 12;

void glPrintString(void *font, char *str)
{
	int i,l = strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}

/*** Here is the main thing to work with to change in Depth image and show the skeleton on Depth image frame ***/
void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd) 
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

  	float topLeftX;
  	float topLeftY;
  	float bottomRightY;
  	float bottomRightX;
	float texXpos;
	float texYpos;
       
	if(!bInitialized)
	{

		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());

		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

		printf("Initialized depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;

	}
	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;
        //const xn::SceneMetaData smd;
	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();

	// Calculate the accumulative histogram
	memset(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0)
			{
				g_pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dmd.Data();
	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = g_pDepthHist[nValue];

						pDestImage[0] = nHistValue * Colors[nColorID][0]; 
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}

	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dmd.XRes(),dmd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);
}

/*** Here is the main thing to work with to change in RGB image and show the skeleton on RGB image frame ***/
void kinect_display_drawRgbMapGL(const xn::ImageMetaData& imd, AttentionMap &attention_map)//, ObjectsFinderBase &objectsFinder) 
{
	static bool bInitialized = false;	
	static GLuint rgbTexID;
	static unsigned char* pRgbTexBuf;
	static int texWidth, texHeight;

  	float topLeftX;
  	float topLeftY;
  	float bottomRightY;
  	float bottomRightX;
	float texXpos;
	float texYpos;
       
	if(!bInitialized)
	{

		texWidth =  getClosestPowerOfTwo(imd.XRes());
		texHeight = getClosestPowerOfTwo(imd.YRes());

		printf("Initializing RGB texture: width = %d, height = %d\n", texWidth, texHeight);
		rgbTexID = initTexture((void**)&pRgbTexBuf,texWidth, texHeight) ;

		printf("Initialized RGB texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = imd.XRes();
		topLeftY = 0;
		bottomRightY = imd.YRes();
		bottomRightX = 0;
		texXpos =(float)imd.XRes()/texWidth;
		texYpos  =(float)imd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;

	}
	unsigned int nX = 0;
	unsigned int nY = 0;
	XnUInt16 g_nXRes = imd.XRes();
	XnUInt16 g_nYRes = imd.YRes();
#if 0
	int sizes[2] = {g_nYRes, g_nXRes};
	cv::Mat objectsMat = objectsFinder.detectObjects(cv::Mat(2, sizes, CV_8UC3, (void*) imd.RGB24Data()));

	unsigned char* pDestImage = pRgbTexBuf;
	if (g_bDrawPixels)
	{
		for (int r = 0; r < objectsMat.rows; ++r)
		{
			unsigned char* r_ptr = objectsMat.ptr<unsigned char>(r);

			for (int c = 0; c < objectsMat.cols; ++c)
			{
				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;

				if (g_bDrawBackground)
				{
					pDestImage[0] = r_ptr[0];
					pDestImage[1] = r_ptr[1];
					pDestImage[2] = r_ptr[2];
				}

				r_ptr+=3;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	#endif

	unsigned char* pDestImage = pRgbTexBuf;
	const XnRGB24Pixel* pRgb = imd.RGB24Data();

	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground)
				{

					pDestImage[0] = pRgb->nRed; 
					pDestImage[1] = pRgb->nGreen;
					pDestImage[2] = pRgb->nBlue;
				}

				pRgb++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pRgbTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}


	attention_map.overlay(pRgbTexBuf, texWidth, texHeight);	


	glBindTexture(GL_TEXTURE_2D, rgbTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pRgbTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(imd.XRes(),imd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);
}

//Draw the line between two joints
void DrawLimb(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	depthGenerator.ConvertRealWorldToProjective(2, pt, pt);
       
	glVertex3i(pt[0].X, pt[0].Y, 0);
	glVertex3i(pt[1].X, pt[1].Y, 0);
}

//Draw ecah joints
void DrawJoint(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint)
{
    char strLabel[50] = "";
  xnOSMemSet(strLabel, 0, sizeof(strLabel));

 if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}
     XnSkeletonJointPosition joint;
     userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);
      if (joint.fConfidence < 0.5){
           return;}
      
        XnPoint3D pt[2];
	pt[0] = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, pt, pt);
        glVertex2f(pt[0].X, pt[0].Y);
}

//Define all the joints name in each joints
const char *GetJointName (XnSkeletonJoint eJoint)
{
                     
	switch (eJoint)
	{
	case XN_SKEL_HEAD: return "head";
	case XN_SKEL_NECK: return "neck";
        case XN_SKEL_LEFT_SHOULDER: return "left shoulder";
	case XN_SKEL_LEFT_ELBOW: return "left elbow";
	case XN_SKEL_RIGHT_SHOULDER: return "right shoulder";
	case XN_SKEL_RIGHT_ELBOW: return "right elbow";
	case XN_SKEL_TORSO: return "torse";
	case XN_SKEL_LEFT_HIP: return "left hip";
	case XN_SKEL_LEFT_KNEE: return "left knee";
	case XN_SKEL_LEFT_FOOT: return "left foot";
	case XN_SKEL_RIGHT_KNEE: return "right knee";
	case XN_SKEL_RIGHT_FOOT: return "right foot";
	case XN_SKEL_RIGHT_HAND: return "right hand";
	case XN_SKEL_LEFT_HAND: return "left hand";
	
	};

	return "Joint";
}

//Draw each points in all the joints
void DrawPoint(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint,
		ofstream &x_file, bool addComma=true)
{
    char strLabel[50] = "";
    xnOSMemSet(strLabel, 0, sizeof(strLabel));

    if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}


    if (g_bPrintID){ 
     	XnSkeletonJointPosition joint;
     	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

      	if (joint.fConfidence < 0.5){
		x_file << "[,]" << (addComma? ", " : "");
           	return;
   	}
	else {

		XnPoint3D pt[2];
		pt[0] = joint.position;
		depthGenerator.ConvertRealWorldToProjective(1, pt, pt);
		glVertex2f(pt[0].X, pt[0].Y);
		sprintf(strLabel, "%s (%.0f, %.0f) ", GetJointName(eJoint), pt[0].X,pt[0].Y);
		glColor3f(1.f,1.f,1.f);
		glRasterPos2i(pt[0].X, pt[0].Y);
		glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);   

        	x_file << "[" << pt[0].X << ", " << pt[0].Y << "]";
		if (addComma) x_file << ", ";
	}
      
    }
}

//Calculate 3D distance between two points inorder to evaluate threshold in prediction
void Distance3D(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	char strLabel[50] = "";
  	xnOSMemSet(strLabel, 0, sizeof(strLabel));

    if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}


	XnVector3D v;
	v.X = joint1.position.X - joint2.position.X;
	v.Y = joint1.position.Y - joint2.position.Y;
	v.Z = joint1.position.Z - joint2.position.Z;
	float distance3D = sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);

	float dist = sqrt(distance3D * distance3D) * 0.001f;
	sprintf(strLabel, " Distance between %s", GetJointName(eJoint1), "and %s", GetJointName(eJoint2),":(%.03fm) ", dist);
    glColor3f(1.f,1.f,1.f);
	glRasterPos2i(20, (eJoint2 == XN_SKEL_RIGHT_HAND)? 80 : 110);
	glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);	
        
}


//Evaluate the velocity of both left and right hand
void Showvelocity(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint)
{

  char strLabel[50] = "";
  xnOSMemSet(strLabel, 0, sizeof(strLabel));
  vector<double> joint_x;
  vector<double> joint_y;
  double jnt_x =0, jnt_y=0;

 if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

       
     XnSkeletonJointPosition joint;
     userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

      if (joint.fConfidence < 0.5){
           return;
        }
            
        XnPoint3D pt[2];
	pt[0] = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, pt, pt);
        glVertex2f(pt[0].X, pt[0].Y);

        double tmp_x =pt[0].X;
        double tmp_y =pt[0].Y;  
       
        double posx_diff = tmp_x - jnt_x;
        double posy_diff = tmp_y - jnt_y;

        jnt_x = tmp_x;
        jnt_y = tmp_y;
     
        double dist = sqrt(posx_diff*posx_diff + posy_diff*posy_diff);
        
        double vel_x = posx_diff * fps;
        double vel_y = posy_diff * fps;
        double vel =  sqrt(vel_x*vel_x + vel_y*vel_y);
        double velocity = dist/fps;
        

        joint_x.push_back(vel_x);
        joint_y.push_back(vel_y);      
           
} 

//Function Struct History is called from trajectory.h (file)
History g_RightHandPositionHistory;
History g_LeftHandPositionHistory;

bool GetHistoryForJoint (XnSkeletonJoint eJoint, History **history) {
	switch (eJoint)
	{
	case XN_SKEL_RIGHT_HAND:
		*history = &g_RightHandPositionHistory;
		break;

	case XN_SKEL_LEFT_HAND:
		*history = &g_LeftHandPositionHistory;
		break;

	default:
		*history = 0;
		return false;
	};

	return true;
}

//Draw hand trajectory at each frame
void handtrajectory(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint, bool updateHistory)
{
	XnSkeletonJointPosition joint;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5){
           return;
        }
            
        XnPoint3D pt_world, pt_screen;
	pt_world = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, &pt_world, &pt_screen);
      
	History *history;
	if (GetHistoryForJoint (eJoint, &history) == false) return;

	if (updateHistory) history->StoreValue (pt_world, pt_screen); // store value in the history

	// Visualize history
	//
	XnFloat pVertexBuffer [HISTORY_DRAW_SIZE * sizeof (float) * 3];
	XnFloat *pVertex = pVertexBuffer;

	// Prepare vertex buffer for drawing
	XnPoint3D pt;
	for (int k = 0; k < history->Size(); ++k) {
		history->GetValueScreen (k, pt);

		*pVertex++ = pt.X;
		*pVertex++ = pt.Y;
		*pVertex++ = 0.0f;
	}

	glColor3f(0.f, 1.f, 0.f);
	glVertexPointer(3, GL_FLOAT, 0, pVertexBuffer);

	// draw trajectory
	glLineWidth(2);
	glDrawArrays(GL_LINE_STRIP, 0, history->Size());

	// draw history points
	glPointSize(8);
	glDrawArrays(GL_POINTS, 0, history->Size());
}

std::vector<XnPoint3D> g_temp_points;

//Draw skeleton in each frame
void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                  xn::DepthGenerator& depthGenerator, bool isDepthPass)
{
	char strLabel[100];
	XnUserID aUsers[3];
	XnUInt16 nUsers = 3;
        GLfloat width = 3;

	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		// Update targets for hand history objects
		{
			XnSkeletonJointPosition headJoint;
			userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, headJoint);//////

			XnPoint3D pt_world, pt_screen, pt_world1, pt_world2;
			pt_world = headJoint.position;
			//pt_world1.X = 320;
			//pt_world1.Y = 240;
			//pt_world2.X = 100;
			//pt_world2.Y = 100;
			depthGenerator.ConvertRealWorldToProjective(1, &pt_world, &pt_screen);
			//depthGenerator.ConvertRealWorldToProjective(1, &pt_world1, &pt_screen);
			//depthGenerator.ConvertRealWorldToProjective(1, &pt_world2, &pt_screen);

			g_LeftHandPositionHistory.SetTarget(pt_world, pt_screen);
			//g_RightHandPositionHistory.SetTarget(pt_world, pt_screen);

			//g_LeftHandPositionHistory.SetTarget(pt_world1, pt_screen);
			//g_RightHandPositionHistory.SetTarget(pt_world1, pt_screen);

			//g_LeftHandPositionHistory.SetTarget(pt_world2, pt_screen);
			//g_RightHandPositionHistory.SetTarget(pt_world2, pt_screen);
		}

		char filename[256];
		sprintf (filename, "JointPositionData:%02d.csv", i);
		ofstream csv_file;
		csv_file.open (filename, ios::app);
                 
		if (g_bPrintID)
		{
			XnPoint3D com;
                  	userGenerator.GetCoM(aUsers[i], com);
			depthGenerator.ConvertRealWorldToProjective(1, &com, &com);// I need to change with image generator
                        glVertex2f(com.X, com.Y);
                        float tmpCOM_x =com.X;
                        float tmpCOM_y =com.Y;     
                      
                        // Calculate the distance from the qrcode to camera
                        float dist = com.Z /1000.0f;       
                        sprintf(strLabel, "Distance :(%.1fm) ", dist); 
                        glColor3f(1.f,1.f,1.f);
                        glRasterPos2i(20, 60);
                        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);

			sprintf(strLabel, "Hand speeds (L/R) :(%.1f/%.1f) ", g_LeftHandPositionHistory.Speed(), g_RightHandPositionHistory.Speed()); 
                        glColor3f(1.f,1.f,1.f);
                        glRasterPos2i(20, 130);
                        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
				
			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState)
			{
				// Tracking
				sprintf(strLabel, "%d", aUsers[i]);
			}
			else if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				sprintf(strLabel, "%d - Tracking", aUsers[i]);

                           
			}
			else if (userGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				sprintf(strLabel, "%d - Calibrating...", aUsers[i]);
			}
			else
			{
				// Nothing
				sprintf(strLabel, "%d - Looking for pose", aUsers[i]);
			}


			glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

			glRasterPos2i(com.X, com.Y);
			glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
		}
                       
                                              
		if (g_bDrawSkeleton && userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
		  
                  	
            glLineWidth(width);
            glBegin(GL_LINES);
			glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);
                       
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);

			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);

			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

			DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
            glEnd();                        

            glBegin(GL_POINTS);
            glColor3f(1.f, 0.f, 0.f);
            glPointSize(1000.0);
                        
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD);
                          
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW);
                           
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_FOOT);
                          
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_FOOT);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND);
                           DrawJoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND);

			glEnd();
			
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD, csv_file);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, csv_file);
                           
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW, csv_file);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_FOOT, csv_file);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_FOOT, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND, csv_file);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND, csv_file, false);
                          
                           Showvelocity(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND);
                           Showvelocity(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND);
               
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND, isDepthPass);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND, isDepthPass);

                           Distance3D(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD, XN_SKEL_RIGHT_HAND);
			   Distance3D(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD, XN_SKEL_LEFT_HAND);
                          
			  DrawCircle(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND, 10, g_RightHandPositionHistory.Color());
			  DrawCircle(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND, 10, g_LeftHandPositionHistory.Color());

			   if (g_RightHandPositionHistory.IsNearTarget())
			   {
				g_temp_points.clear();
				g_RightHandPositionHistory.GetApproachCurveControlPoints(g_temp_points);
				DrawBezierCurve(g_temp_points);
				
				sprintf(strLabel, " PREDICTING.."); 
		                glColor3f(1.f,0.f,0.f);
		                glRasterPos2i(280, 400);
		                glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);

			   }
			   if (g_LeftHandPositionHistory.IsNearTarget())
			   {
				g_temp_points.clear();
				g_LeftHandPositionHistory.GetApproachCurveControlPoints(g_temp_points);
			   	DrawBezierCurve(g_temp_points);
				
				sprintf(strLabel, "PREDICTION: DRINKING"); 
		                glColor3f(1.f,0.f,0.f);
		                glRasterPos2i(280, 400);
		                glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
			   }

			   if (!g_RightHandPositionHistory.IsStationary())
			   {
				sprintf(strLabel, " LEFT HAND IS MOVING"); 
		                glColor3f(1.f,0.f,0.f);
		                glRasterPos2i(280, 420);
		                glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
			   }
			   if (!g_LeftHandPositionHistory.IsStationary())
			   {
				sprintf(strLabel, "RIGHT HAND IS MOVING"); 
		                glColor3f(1.f,0.f,0.f);
		                glRasterPos2i(280, 440);
		                glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
			   }
		}

		csv_file.close();
   		
 
}
 
}

        
	

