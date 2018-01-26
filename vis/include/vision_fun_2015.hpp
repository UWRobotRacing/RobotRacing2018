#ifndef __VISION_FUN_2015
#define __VISION_FUN_2015
// Vision support functions used in 2015 robot racing

// includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void multithresh(const cv::Mat &Im, const cv::Mat &bounds, cv::Mat &Im_Log);

void adapthresh_white_HSV(const cv::Mat &Im, const cv::Scalar bounds, int patchsize, cv::Mat &Im_Log);

cv::Mat skeletonize(const cv::Mat &Im);

cv::Mat filtersize(const cv::Mat &Im1, int minsize);

#endif
