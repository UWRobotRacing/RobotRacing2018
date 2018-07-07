/** @file thresholding.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef __THRESHOLDING_HPP
#define __THRESHOLDING_HPP

// includes
using namespace std;    //required for gpu
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>

void Multithreshold(const cv::Mat &input_image, const cv::Mat &bounds, cv::Mat &output_image);
void FindWhite(const cv::Mat &input_image, const cv::Scalar bounds, int patch_size, cv::Mat &output_image);
cv::Mat Skeletonize(const cv::Mat &image);
cv::Mat GetContours(const cv::Mat &image, int min_size);
#endif //__THRESHOLDING_HPP

/*
void Multithreshold(const cv::gpu::GpuMat &input_image, const cv::gpu::GpuMat &bounds, cv::gpu::GpuMat &output_image);
void FindWhite(const cv::gpu::GpuMat &input_image, const cv::Scalar bounds, int patch_size, cv::gpu::GpuMat &output_image);
cv::Mat Skeletonize(const cv::gpu::GpuMat &image);
cv::Mat GetContours(const cv::gpu::GpuMat &image, int min_size);
#endif //__THRESHOLDING_HPP
*/