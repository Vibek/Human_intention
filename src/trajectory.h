/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef TRAJECTORY_
#define TRAJECTORY_

#include <GL/glut.h>
#include <XnV3DVector.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "KinectDisplay.h"
#include "trajectory.h"

#define HISTORY_SIZE		10
#define HISTORY_DRAW_SIZE	HISTORY_SIZE

#define TARGET_HEAD		0

struct History {
	History (int size = HISTORY_SIZE)
	: m_max_size(size)
	, m_records(m_max_size)
	, m_size(0)
	, m_curr_pos(m_max_size)
	{
		for (int i = 0; i < NumObjectsOfInterest + 1; ++i)
		{
			m_target_world[i].X = m_target_world[i].Y = m_target_world[i].Z = 0.0f;
			m_target_screen[i] = m_target_world[i];
		}
	}

	int Size() { return m_size; }

	void StoreValue (const XnPoint3D &pt_world, const XnPoint3D &pt_screen)
	{
		if (--m_curr_pos < 0) m_curr_pos = m_max_size - 1;
		m_records [m_curr_pos].value_world = pt_world;
		m_records [m_curr_pos].value_screen = pt_screen;
		m_records [m_curr_pos].time = glutGet(GLUT_ELAPSED_TIME);

		if (++m_size > m_max_size) m_size = m_max_size;
	}

	XnPoint3D GetCurrentWorldPosition() { return m_records[m_curr_pos].value_world; } // in millimeters
	XnV3DVector GetCurrentScreenPosition() { return m_records[m_curr_pos].value_screen; } // in pixels

	XnPoint3D GetTargetWorldPosition(int targetIndex=TARGET_HEAD) { return m_target_world[targetIndex + 1]; } // in millimeters
	XnV3DVector GetTargetScreenPosition(int targetIndex=TARGET_HEAD) { return m_target_screen[targetIndex + 1]; } // in pixels

	bool GetValueScreen (int index, XnPoint3D &pt)
	{
		if (index < 0 || index > m_size) return false;

		pt = m_records [(m_curr_pos + index) % m_max_size].value_screen;
		return true;
	}

	bool GetValueWorld (int index, XnPoint3D &pt)
	{
		if (index < 0 || index > m_size) return false;

		pt = m_records [(m_curr_pos + index) % m_max_size].value_screen;
		return true;
	}

	XnV3DVector GetCurrentDirectionScreen()
	{
		XnV3DVector v1 = m_records[m_curr_pos].value_screen;
		XnV3DVector v2 = m_records[(m_curr_pos + 1) % m_max_size].value_screen;
		XnV3DVector v = v1 - v2;
		v.Normalize();
		return v;
	}

	XnV3DVector GetTargetApproachVectorScreen(int targetIndex=TARGET_HEAD)
	{
		XnV3DVector v1 = m_target_screen[targetIndex + 1];
		XnV3DVector v2 = m_records[m_curr_pos].value_screen;
		XnV3DVector v = v1 - v2;
		v.Normalize();
		return v;
	}

	float Speed() // in pixels per second
	{
		if (m_size < 2) return 0.0f;

		Record &last = m_records[m_curr_pos];
		Record &prev = m_records[(m_curr_pos + 1) % m_max_size];

		XnV3DVector lastV = last.value_screen;
		XnV3DVector prevV = prev.value_screen;

		return (lastV - prevV).Magnitude() / ((last.time - prev.time) * 0.001f);
	}

	bool IsStationary()  {return Speed() < 150;}
			

	bool IsNearTarget(int targetIndex=TARGET_HEAD) { return GetDistanceToTarget(targetIndex) < 0.5f; }

	XnFloat *Color()
	{
		static XnFloat colors[][3] = {
			{1, 1, 0}, // yellow
			{0, 0, 1}, // blue
			{1, 0, 0}  // red
		};

		if (IsNearTarget()){ 
			return colors[2];
			}

		else if (IsStationary()) {
			return colors[0];
			
				}

		else {  return colors[1];
			
				}
	}

	void SetTarget (int targetIndex, XnPoint3D target_world, XnPoint3D target_screen) // position in millimeters
	{
		m_target_world[targetIndex + 1] = target_world;
		m_target_screen[targetIndex + 1] = target_screen;
	}

	float GetDistanceToTarget(int targetIndex=TARGET_HEAD) // distance in meters
	{
           	XnPoint3D pos = GetCurrentWorldPosition();
		XnVector3D v;
		v.X = m_target_world[targetIndex + 1].X - pos.X;
		v.Y = m_target_world[targetIndex + 1].Y - pos.Y;
		v.Z = m_target_world[targetIndex + 1].Z - pos.Z;
		float distance3D = sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);              
		return distance3D * 0.001f;
	}

	void GetApproachCurveControlPoints(int targetIndex, std::vector<XnPoint3D> &points)
	{
		points.push_back(GetCurrentScreenPosition());
		points.push_back(GetCurrentScreenPosition() + GetCurrentDirectionScreen() * Speed() * 1.0f);
		points.push_back(GetTargetScreenPosition(targetIndex) - GetTargetApproachVectorScreen(targetIndex) * 10.0f);
		points.push_back(GetTargetScreenPosition(targetIndex));
	}

	void GetPointsNewerThanTime(int timeMilliSec, std::vector<XnPoint3D> &points)
	{
		int count = Size();
		for (int index = 0; index < count; ++index)
		{
			Record &rec = m_records [(m_curr_pos + index) % m_max_size];
			if (rec.time >= timeMilliSec)
			{
				points.push_back(rec.value_screen);
			}
		}
	}

	int GetNumObjectsOfInterest() { return NumObjectsOfInterest; }

private:
	const int m_max_size;

	struct Record
	{
		XnPoint3D value_world; // world position in millimeters
		XnPoint3D value_screen;// screen position in pixels
		int	  time; //milliseconds
	};
	std::vector<Record> m_records;

	int m_size;
	int m_curr_pos;

	XnPoint3D m_target_world[1 + NumObjectsOfInterest]; // in millimeters
	XnPoint3D m_target_screen[1 + NumObjectsOfInterest]; // in pixels
};

bool GetHistoryForJoint (XnSkeletonJoint eJoint, History **history);

#endif

