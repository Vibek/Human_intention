/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/
#include <GL/glut.h>

#include "attention_map.hpp"
#include "trajectory.h"
#include "BezierCurve.hpp"
#include "utils.hpp"

int g_kernel_size = 15;
float g_fade_time = 2.0;
float g_base_intensity = 0.1;
float g_max_transparency = 0.6;
bool g_linear_kernel = false;


void AttentionMap::init(int fps, int width, int height)
{
	m_heatmap = cv::Mat::zeros(height, width, CV_32FC1);
	m_ones = cv::Mat::ones(height, width, CV_32F);
	m_zeros = cv::Mat::zeros(height, width, CV_32F);

	m_fade_mat = cv::Mat::ones(height, width, CV_32F);
	// Determine how much to fade the heatmap values by each frame
	m_fade_mat.setTo((1.0 / fps) / g_fade_time);
	// Create heatmap kernel
	
	create_kernel();

	m_last_update_time = 0;
}

std::vector<XnPoint3D> g_heat_points;
std::vector<XnPoint3D> g_bezier_ctrl_pts;

void AttentionMap::CollectHeatPoints(XnSkeletonJoint eJoint, std::vector<XnPoint3D> &heat_points)
{
	History *history;
	if (GetHistoryForJoint (eJoint, &history))
	{
		if (history->IsStationary() == false)
		{
			history->GetPointsNewerThanTime (m_last_update_time, g_heat_points);
		}

		if (history->IsNearTarget())
		{
			g_bezier_ctrl_pts.clear();
			history->GetApproachCurveControlPoints(g_bezier_ctrl_pts);

			BezierCurveGen curve(g_bezier_ctrl_pts);

			XnPoint3D pt;
			while(curve.next_point(pt))
			{
				g_heat_points.push_back(pt);
			};
		}
	}
}

void AttentionMap::update()
{
	g_heat_points.clear();
	CollectHeatPoints(XN_SKEL_LEFT_HAND, g_heat_points);///////
	CollectHeatPoints(XN_SKEL_RIGHT_HAND, g_heat_points);///////

	for (int k = 0; k < g_heat_points.size(); ++k)
	{
		XnPoint3D &pt = g_heat_points[k];
		heat_point(pt.X, pt.Y);
	}

	int current_time = glutGet(GLUT_ELAPSED_TIME);
	m_last_update_time = current_time;
}

void AttentionMap::fade()
{
	// Fade some of the values in the matrix	
	m_heatmap -= m_fade_mat;
	m_heatmap = max(m_zeros, m_heatmap);
}

//============================================================================
// heat_point
//============================================================================

void AttentionMap::heat_point(int x, int y)
{
	// Make sure the coordinates are in bounds
	if (x < 0 || y < 0 || x >= m_heatmap.cols || y >= m_heatmap.rows)
	{
		return;
	}

	// Only update a small portion of the matrix
	const int g_kernel_half = g_kernel_size / 2;
	const int fixed_x = x - g_kernel_half;
	const int fixed_y = y - g_kernel_half;
	const int roi_l = max(fixed_x, 0);
	const int roi_t = max(fixed_y, 0);
	const int roi_w = min(fixed_x + g_kernel_size, m_heatmap.cols) - roi_l;
	const int roi_h = min(fixed_y + g_kernel_size, m_heatmap.rows) - roi_t;

	cv::Mat roi(m_heatmap(cv::Rect(roi_l, roi_t, roi_w, roi_h)));

	const int groi_l = roi_l - fixed_x;
	const int groi_t = roi_t - fixed_y;
	const int groi_w = roi_w;
	const int groi_h = roi_h;

	cv::Mat roi_gauss(m_kernel(cv::Rect(groi_l, groi_t, groi_w, groi_h)));
	roi += roi_gauss;
}


// Following are in HSV format
cv::Vec3b g_heat_color1 = cv::Vec3b(0, 255, 255); // Red
cv::Vec3b g_heat_color2 = cv::Vec3b(170, 255, 255); // Blue

//============================================================================
// overlay_heatmap
//============================================================================

/* Draws the heatmap on top of a frame. The frame must be the same size as
 * the heatmap. 
 */
void AttentionMap::overlay(unsigned char* pDestImage, int imageWidth, int imageHeight)
{
	update();

	// Make sure all values are capped at one
	m_heatmap = min(m_ones, m_heatmap);

	cv::Mat temp_map;
	blur(m_heatmap, temp_map, cv::Size(15, 15));

	for (int r = 0; r < m_heatmap.rows; ++r)
	{
		
		float* h_ptr = temp_map.ptr<float>(r);
		for (int c = 0; c < m_heatmap.cols; ++c)
		{
			const float heat_mix = h_ptr[c];
			if (heat_mix > 0.0)
			{
				// in BGR
				const cv::Vec3b i_color = cv::Vec3b(pDestImage[0], pDestImage[1], pDestImage[2]);

				const cv::Vec3b heat_color = 
					hsv_to_bgr(interpolate_hsv(g_heat_color1, g_heat_color2, heat_mix));

				const float heat_mix2 = std::min(heat_mix, g_max_transparency);

				const cv::Vec3b final_color = interpolate(i_color, heat_color, heat_mix2);
				
				pDestImage[0] = final_color[0];
				pDestImage[1] = final_color[1];
				pDestImage[2] = final_color[2];
			}

			pDestImage+=3;
		}

		pDestImage += (imageWidth - m_heatmap.cols) *3;
	}

	fade();
}

//============================================================================
// create_kernel
//============================================================================

/* Create the heatmap kernel. This is applied when heat_point() is called. 
 */
void AttentionMap::create_kernel()
{
	if (g_linear_kernel)
	{
		// Linear kernel
		const float max_val = 1.0 * g_base_intensity;
		const float min_val = 0.0;
		const float interval = max_val - min_val;

		const int center = g_kernel_size / 2 + 1;
		const float radius = g_kernel_size / 2;

		m_kernel = cv::Mat::zeros(g_kernel_size, g_kernel_size, CV_32F);
		for (int r = 0; r < g_kernel_size; ++r)
		{
			float* ptr = m_kernel.ptr<float>(r);
			for (int c = 0; c < g_kernel_size; ++c)
			{
				// Calculate the distance from the center	
				const float diff_x = static_cast<float>(abs(r - center));
				const float diff_y = static_cast<float>(abs(c - center));
				const float length = sqrt(diff_x*diff_x + diff_y*diff_y);
				if (length <= radius)
				{
					const float b = 1.0 - (length / radius);
					const float val = b*interval + min_val;
					ptr[c] = val;
				}
			}
		}
	}
	else
	{
		// Gaussian kernel
		cv::Mat coeffs = cv::getGaussianKernel(g_kernel_size, 0.0, CV_32F)*150*g_base_intensity;
		m_kernel = coeffs * coeffs.t();
	}
}
