/** @file thresholding.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef __THRESHOLDING_HPP
#define __THRESHOLDING_HPP

// includes
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void Multithreshold(const cv::Mat &input_image, const cv::Mat &bounds, cv::Mat &output_image);
void FindWhite(const cv::Mat &input_image, const cv::Scalar bounds, int patch_size, cv::Mat &output_image);
cv::Mat GetContours(const cv::Mat &image, int min_size);
#endif //__THRESHOLDING_HPP