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
	const double MORPH_KERNEL_SIZE = 3;
    const double MIN_SHADOW_AREA = 30;
    const double MASK_DILATION = 6;
    const double EXPANDED_SHADOW_MASK_DILATION = 10;
    const double EXPANDED_EDGE_MASK_DILATION = 8;
    const double MED_FILTER_KERNEL_SIZE = 15;

	// Get the mean of L, a, b planes
	cv::Mat shadow_lab_image;
	cvtColor(input_image, shadow_lab_image, CV_BGR2Lab, 0);

	std::vector<cv::Mat> channels;
	split(shadow_lab_image, channels);
	cv::Scalar mean_l, stdev_l;
	meanStdDev(channels[0], mean_l, stdev_l);
	cv::Scalar mean_a = mean(channels[1]);
	cv::Scalar mean_b = mean(channels[2]);

	// Extract shadows using best method based on mean values
	cv::Mat shadow_pixels_mask;
	double shadow_threshold = mean_l[0] - (stdev_l[0] / THRESHOLD_TOLERANCE);

	if (mean_a[0] + mean_b[0] > 256) {
		cv::inRange(shadow_lab_image, cv::Scalar(0, 0, 0), cv::Scalar(shadow_threshold, 1, 1), shadow_pixels_mask);
	} else {
		cv::Mat mask_l, mask_b;
		cv::inRange(shadow_lab_image, cv::Scalar(0, 0, 0), cv::Scalar(shadow_threshold, 1, 1), mask_l);
		cv::inRange(shadow_lab_image, cv::Scalar(0, 0, 0), cv::Scalar(1, 1, shadow_threshold), mask_b);
		shadow_pixels_mask = mask_l | mask_b;
	}

	// Morphological operations to clean up misclassified pixels
	cv::Mat shadow_mask_morph;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));

	cv::morphologyEx(shadow_pixels_mask, shadow_mask_morph, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(shadow_mask_morph, shadow_mask_morph, cv::MORPH_CLOSE, kernel);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(shadow_mask_morph, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++) {
		double area = cv::contourArea(contours[i]);

		// Fill in contours with area smaller than min. shadow area
		if (area < MIN_SHADOW_AREA) {
			cv::drawContours(shadow_mask_morph, contours, i, CV_RGB(0, 0, 0), CV_FILLED);
		}
	}

	kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MASK_DILATION, MASK_DILATION));
	cv::dilate(shadow_mask_morph, shadow_mask_morph, kernel);

	// Shadow regions
	cv::Mat connected_components;
	int num_components = cv::connectedComponents(shadow_mask_morph, connected_components, 4);

	// Shadow removal
	cv::Mat corrected_image = input_image.clone();

	for (int i = 1; i <= num_components; i++) {
		cv::Mat component_mask;
		cv::inRange(connected_components, cv::Scalar(i), cv::Scalar(i), component_mask);

		cv::Mat dilated_component_mask;
		kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(EXPANDED_SHADOW_MASK_DILATION, EXPANDED_SHADOW_MASK_DILATION));
		cv::dilate(component_mask, dilated_component_mask, kernel);
		cv::bitwise_not(dilated_component_mask, component_mask, dilated_component_mask);

		// Get mean RGB in masks
		cv::Scalar outer_mean_rgb = mean(corrected_image, dilated_component_mask);
		cv::Scalar inner_mean_rgb = mean(corrected_image, component_mask);

		cv::Scalar rgb_ratios(outer_mean_rgb[0] / inner_mean_rgb[0],
							outer_mean_rgb[1] / inner_mean_rgb[1],
							outer_mean_rgb[2] / inner_mean_rgb[2]);

		if (std::isinf(rgb_ratios[0])) {
			rgb_ratios[0] = 1;
		}
		if (std::isinf(rgb_ratios[1])) {
			rgb_ratios[1] = 1;
		}
		if (std::isinf(rgb_ratios[2])) {
			rgb_ratios[2] = 1;
		}

		// Multiply shadow region by ratios to remove shadow
		cv::Mat component_region;
		corrected_image.copyTo(component_region, component_mask);
		component_region = component_region.mul(rgb_ratios);
		component_region.copyTo(corrected_image, component_mask);
	}
}