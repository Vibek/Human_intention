/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#include <GL/glut.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <stdio.h>
#include "KinectDisplay.h"
//#include <vector.h>
//#include <stdio.h>
#include <stdlib.h>
#include <math.h>


XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;
XnBool g_bPrintFrameID = FALSE;

extern xn::ImageGenerator ImageGenerator;

using std::string;

#define PI 3.14159265359
#define fps 30
using namespace std;

#define MAX_DEPTH 10000

// focal length(calculated before) and test distance
float f = 525.0, D = 45.0;   

float g_pDepthHist[MAX_DEPTH];
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
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
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

/*** Here is the main thing to work with to change in RGB image and show the skeleton on RGB image frame ***/

void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd) // I need to check with imageMetaData
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
        //depthGenerator.GetAlternativeViewPointCap().SetViewPoint(ImageGenerator);
	glVertex3i(pt[0].X, pt[0].Y, 0);
	glVertex3i(pt[1].X, pt[1].Y, 0);
}

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
        //ImageGenerator.GetAlternativeViewPointCap().SetViewPoint(UserGenerator);
        //DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(ImageGenerator);
        glVertex2f(pt[0].X, pt[0].Y);
        //glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
}

void DrawPoint(xn::UserGenerator& userGenerator,
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
    
       ofstream x_file;
       x_file.open ("JointPositionData.csv", ios::app);

  if (g_bPrintID){ 
     //PVector joint = new PVector();
     XnSkeletonJointPosition joint;
     userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);
     //println(joint);
      if (joint.fConfidence < 0.5){
           return;
    }
      
        XnPoint3D pt[2];
	pt[0] = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, pt, pt);
        glVertex2f(pt[0].X, pt[0].Y);
        sprintf(strLabel, "Joint (%.0f, %.0f) ", pt[0].X,pt[0].Y);
        glColor3f(1.f,1.f,1.f);
        glRasterPos2i(pt[0].X, pt[0].Y);
        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
      
        double posx =pt[0].X;
        double posy =pt[0].Y;  
       
        double posx_diff = posx - jnt_x;
        double posy_diff = posy - jnt_y;

        jnt_x = posx;
        jnt_y = posy;

        joint_x.push_back(posx_diff);
        joint_y.push_back(posy_diff);

        //cout <<"JOINT VALUE PUSHED INTO VECTOR" << jnt_x <<","<< jnt_y<<endl;

        vector<double>::iterator j = joint_y.begin();

         for (vector<double>::iterator i = joint_x.begin(); i != joint_x.end();++i)
         {
          x_file << "Pos" << *i <<"," <<*j <<"." <<endl;
          j++;
        }      
    }
    x_file.close();   
  }

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
        
        //double factor = vel/dist;
        double vel_x = posx_diff * fps;
        double vel_y = posy_diff * fps;
        double vel =  sqrt(vel_x*vel_x + vel_y*vel_y);
        double velocity = dist/fps;
        

        joint_x.push_back(vel_x);
        joint_y.push_back(vel_y);
      

        //cout <<"JOINT VALUE PUSHED INTO VECTOR" << velocity<<endl;
        sprintf(strLabel, "Velocity :(%.0lf) ", velocity); 
 
        glColor3f(1.f,1.f,1.f);
        glRasterPos2i(20, 20);
        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
 
        
} 


void handtrajectory(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint)
{
         
       // previous positions per hand
       std::list<XnPoint3D> m_History;
        //std::list<XnPoint3D> f_History;
         XnFloat *m_pfPositionBuffer;
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
            
        XnPoint3D pt[2], pnt, point;
        point.X=-1; point.Y=-1;
	pt[0] = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, pt, pt);
                             
          pnt.X = pt[0].X - point.X;
          pnt.Y = pt[0].Y - point.Y;  
         
          point.X = pnt.X;
          point.Y = pnt.Y;
    
        m_History.push_front(pt[0]);
        //pnt(m_History.size()-1);
        //f_History.push_front(pnt);
	std::list<XnPoint3D>::const_iterator PointIterator;
            XnUInt32 nHistory;
           m_pfPositionBuffer = new XnFloat[nHistory*3];
       
	// Go over each existing hand
	for (PointIterator = m_History.begin();
		PointIterator != m_History.end();
		++PointIterator)
	{                 
                                             
			XnUInt32 i = 0;		
                
              std::list<XnPoint3D>::const_iterator PositionIterator;
            for (PositionIterator = m_History.begin();
		PositionIterator != m_History.end();
		++PositionIterator){

                       // XnFloat m_pfPositionBuffer[i];
                          // Add position to buffer
                        XnPoint3D temp(pt[0]);                       
                  	// Add position to buffer
			m_pfPositionBuffer[3*i] = temp.X;
			m_pfPositionBuffer[3*i + 1] = temp.Y;
			m_pfPositionBuffer[3*i + 2] = 0;//pos.Z();
                       	++i;	       
                   }   

		// Draw buffer:
		glColor3f(0.f, 1.f, 0.f);
		glPointSize(8);
		glVertexPointer(3, GL_FLOAT, 0, m_pfPositionBuffer);
		glDrawArrays(GL_LINE_STRIP, 0, i);
         //}
		glPointSize(8);
		glDrawArrays(GL_POINTS, 0, 1);
		glFlush();
	}
}   

void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                  xn::DepthGenerator& depthGenerator)
{
	char strLabel[50] = "";
	XnUserID aUsers[3];
	XnUInt16 nUsers = 3;
        GLfloat width = 3;
    
	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
                 
		if (g_bPrintID)
		{
			XnPoint3D com;
                  	userGenerator.GetCoM(aUsers[i], com);
			depthGenerator.ConvertRealWorldToProjective(1, &com, &com);// I need to change with image generator
                        glVertex2f(com.X, com.Y);
                        float tmpCOM_x =com.X;
                        float tmpCOM_y =com.Y;         
                      
                        // Calculate the distance from the qrcode to camera
                        float distanceL = (f * D) / tmpCOM_y;
                        float distanceR = (f * D) / tmpCOM_y;                        
                        float dist = sqrt(distanceL*distanceL + distanceR*distanceR);
                        //float dist = ((distanceL + distanceR) / 2)/299.8701;
                        //cout << "Distance:" << dist <<endl;
                        sprintf(strLabel, "Distance :(%.0lfcm) ", dist); 
                        glColor3f(1.f,1.f,1.f);
                        glRasterPos2i(30, 50);
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
                            
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_FOOT);
                          
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_FOOT);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND);
                           DrawPoint(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND);
                          
                           Showvelocity(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND);
                           Showvelocity(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND);
                     /*
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD);
                          
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK);
                           
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW);
                          
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_FOOT);
                          
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_FOOT);*/
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HAND);
                           handtrajectory(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HAND);
                          
		}
 
}
 
}





