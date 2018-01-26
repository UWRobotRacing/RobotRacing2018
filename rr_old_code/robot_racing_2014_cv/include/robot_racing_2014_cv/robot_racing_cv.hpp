#ifndef __ROBOT_RACING_CV_INCLUDED
#define __ROBOT_RACING_CV_INCLUDED
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>

typedef std::vector<cv::Point2i> point_vector_i;

bool compare_blobs(const point_vector_i A, const point_vector_i B);

bool compare_points (const cv::Point2i A, const cv::Point2i B);

void removeScatteredSubvector(point_vector_i &supervector, point_vector_i remove_me);

void findBlobs(const cv::Mat &binary, std::vector < point_vector_i > &blobs);

void findCircleBlob(const std::vector < point_vector_i > blobs, point_vector_i &circle_blob, const double max_eccentricity);

#endif