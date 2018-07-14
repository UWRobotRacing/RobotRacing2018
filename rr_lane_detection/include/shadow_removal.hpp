/** @file shadow_removal.hpp
 *  @author Brian Tran
 *  @competition IARRC 2018
 */
#ifndef __SHADOW_REMOVAL_HPP
#define __SHADOW_REMOVAL_HPP

// includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void RemoveShadows(const cv::Mat &input_image, cv::Mat &output_image);
#endif //__SHADOW_REMOVAL_HPP