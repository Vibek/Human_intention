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
#include <stdlib.h>
#include <math.h>
#include "BezierCurve.hpp"

// Draw the circle in the required joint 

void DrawCircle(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint, float radius, XnFloat *color3f) 
{ 

	XnSkeletonJointPosition joint;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5){
           return;
        }	

    	XnPoint3D pt;
	pt = joint.position;
        depthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);
        float cx = pt.X;
	float cy = pt.Y;
        float r = radius;
	int num_segments = 16;

	glColor3f(color3f[0], color3f[1], color3f[2]);

	glBegin(GL_TRIANGLE_FAN); 
	glVertex2f(cx , cy);
	for(int i = 0; i <= num_segments; i++) 
	{ 
		float theta = 2.0f * 3.1415926f * float(i) / float(num_segments);//get the current angle 

		float x = r * cosf(theta);//calculate the x component 
		float y = r * sinf(theta);//calculate the y component 

		glVertex2f(x +cx , y + cy);//output vertex 

	} 
	glEnd(); 
}


// BezierCurve implementation
//
int factorial(int x, int result = 1) {
if (x == 1 || x ==0) return result; else return factorial(x - 1, x * result);
}


float Bernstein(int n, int j, float t){
return factorial(n)/(factorial(j)* factorial(n-j))*pow(t,j)*pow((1-t),(n-j));
}

BezierCurveGen::BezierCurveGen(const std::vector<XnPoint3D> &controlPoints, int numPoints)
: m_controlPoints(controlPoints)
{
	for(int i = 0; i < numPoints ; i ++){
		m_t.push_back(i*1.0/(numPoints-1));
	}

	m_i = 0;
}

bool BezierCurveGen::next_point(XnPoint3D &pt)
{
	if (m_i == m_t.size())
	{
		return false;
	}

	pt.X = pt.Y = pt.Z = 0.0f;
	for(size_t j = 0; j < m_controlPoints.size(); j++){
		float val = Bernstein(m_controlPoints.size()-1, j, m_t.at(m_i));
		pt.X = pt.X + m_controlPoints.at(j).X *val;
		pt.Y = pt.Y + m_controlPoints.at(j).Y *val;
		pt.Z = pt.Z + m_controlPoints.at(j).Z *val;
	}

	++m_i;
	return true;
}

//


void DrawBezierCurve(const std::vector<XnPoint3D> &controlPoints, int numPoints){
	char strLabel[256];
	sprintf(strLabel, "[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]",
		controlPoints[0].X, controlPoints[0].Y,
		controlPoints[1].X, controlPoints[1].Y,
		controlPoints[2].X, controlPoints[2].Y,
		controlPoints[3].X, controlPoints[3].Y); 
        glColor3f(1.f,1.f,1.f);
        glRasterPos2i(20, 300);
        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);

	glColor3f(0, 0, 0);
	glBegin(GL_LINE_STRIP);
	
	BezierCurveGen curve(controlPoints, numPoints);
	XnPoint3D pt;
	while(curve.next_point(pt))
	{
		glVertex2f(pt.X, pt.Y);
	} 
	glEnd();
}



