#ifndef __ROBOT_RACING_CV_INCLUDED
#define __ROBOT_RACING_CV_INCLUDED
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef std::vector<cv::Point2i> point_vector_i;

bool compare_blobs(const point_vector_i A, const point_vector_i B);

bool compare_points (const cv::Point2i A, const cv::Point2i B);

void removeScatteredSubvector(point_vector_i &supervector, point_vector_i remove_me);

void findBlobs(const cv::Mat &binary, std::vector < point_vector_i > &blobs);

void findCircleBlob(const std::vector < point_vector_i > blobs, point_vector_i &circle_blob, const double max_eccentricity);

void laneClassifier(const cv::Mat &input_image, cv::Mat &output_image);

class NeuralNetLaneClassifier
{
private:
	//private default constructor
	NeuralNetLaneClassifier() {};

	//patch classifier
	bool classify_patch(const cv::Mat &input_vector);

	//Network weights
	cv::Mat net_tx_1;
	cv::Mat net_tx_2;
	cv::Size classifier_window;	

public:
	//Constructor with weight initialization
	NeuralNetLaneClassifier(std::string &net_tx_1_path,
							std::string &net_tx_2_path,
							cv::Size &net_tx1_size,
							cv::Size &net_tx2_size,
							cv::Size &classifier_window_size);

	//Classifier function
	void classify_image(const cv::Mat &input_image, const cv::Mat &mask, cv::Mat &output_image);

	//Accessors
	cv::Mat getMat_net_tx_1();
	cv::Mat getMat_net_tx_2();	
};

#endif