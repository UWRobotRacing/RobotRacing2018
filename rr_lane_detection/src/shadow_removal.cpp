/** @file shadow_removal.cpp
 *  @author Brian Tran
 *  @competition IARRC 2018
 */

#include "shadow_removal.hpp"

#include <opencv2/opencv.hpp>

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
	// return input_image;
}