/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

/* Interpolates between two HSV colors. This function differs from
 * regular interpolation because the hue value is treated specially.
 *
 * H - value 0, range 0-255 -> 0-360 degrees
 * S - value 1, range 0-255 -> 0-1
 * V - value 2, range 0-255 -> 0-1
 *
 * Params:
 *   color1 - The starting color, multiplied by (1.0 - value).
 *   color2 - The ending color, multiplied by value.
 *   value  - The interpolation parameter in the range 0-1.
 *
 * Returns:
 *   A new HSV color that is a mix of color1 and color2.
 */
cv::Vec3b interpolate_hsv(const cv::Vec3b color1, const cv::Vec3b color2, const float value);

/* Generic interpolation function, can be used with any Vec3b, not just color values.
 *
 * Params:
 *   color1 - The starting color, multiplied by (1.0 - value).
 *   color2 - The ending color, multiplied by value.
 *   value  - The interpolation parameter in the range 0-1.
 *
 * Returns:
 *   A new color that is a mix of color1 and color2.
 */
cv::Vec3b interpolate(const cv::Vec3b color1, const cv::Vec3b color2, const float value);

/* Convert an HSV color to an RGB color.
 *
 * algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
 *
 * H - 0-255 -> 0-360 degrees
 * S - 0-255 -> 0-1
 * V - 0-255 -> 0-1
 *
 * Params:
 *   hsv - An HSV color in the above format.
 *
 * Returns:
 *   An RGB space color with the components in BGR order.
 */
cv::Vec3b hsv_to_bgr(const cv::Vec3b& hsv);



#endif // __UTILS_HPP__
