/** @file shadow_removal.cpp
 *  @author Brian Tran
 *  @competition IARRC 2018
 */

#include "shadow_removal.hpp"

/** @brief This function is intended to detect shadows and reduce the difference
 *         in illumination in those regions
 *
 * 	"output_image","input_image", of type CV_8U (unsigned 8 bit int), where
 * 	true == 255 else false == 0
 * 
 *  @param input_image is an image Matrix
 *  @return output_image is the processed output image with the same
 *          width and height as the input matrix
 */
void RemoveShadows(const cv::Mat &input_image, cv::Mat &output_image) {
	// Configurable constants
    const double THRESHOLD_TOLERANCE = 0.8;
    const double MIN_SHADOW_AREA = 30;
    const double MASK_DILATION = 6;
    const double EXPANDED_SHADOW_MASK_DILATION = 10;
    const double EXPANDED_EDGE_MASK_DILATION = 8;
    const double MED_FILTER_KERNEL_SIZE = 15;

	// Get the mean of L, a, b planes
	cv::Mat shadow_lab;
	cvtColor(input_image, shadow_lab, CV_BGR2Lab, 0);

	std::vector<cv::Mat> channels;
	split(shadow_lab, channels);
	cv::Scalar mean_l = mean(channels[0]);
	cv::Scalar mean_a = mean(channels[1]);
	cv::Scalar mean_b = mean(channels[2]);
}