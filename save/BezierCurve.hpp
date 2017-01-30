/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef __BEZIER_CURVE_HPP__
#define __BEZIER_CURVE_HPP__

#include <XnV3DVector.h>
#include <vector>

struct BezierCurveGen
{
	BezierCurveGen(const std::vector<XnPoint3D> &controlPoints, int numPoints = 16);

	bool next_point(XnPoint3D &pt);

private:
	const std::vector<XnPoint3D> &m_controlPoints;
	std::vector<float> m_t;
	int m_i;
};

#endif

