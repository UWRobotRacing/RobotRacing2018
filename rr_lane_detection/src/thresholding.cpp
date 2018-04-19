/** @file thresholding.cpp
 *  @author Matthew Post(mpost)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#include "thresholding.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/** @brief This function is intended to work in the same fashion as CV::inRange
 *         but allows for a cv::Matrix of min and max ranges to be passed, with 
 *         channel wrapping supported
 *
 * 	"output_image","input_image", of type CV_8U (unsigned 8 bit int), where
 * 	true == 255 else false == 0
 * 
 *  @param input_image is an image Matrix
 *  @param bounds is an n by m matrix where n is the number of threshold bands
 *         to be accepted and m is 2*(number of colour channels 
 *         in input_image)
 *  @return output_image is the resulting logical output image with the same
 *          width and height as the input matrix
 */
void Multithreshold(const cv::Mat &input_image, const cv::Mat &bounds, cv::Mat &output_image) {
	//Confirm Dims and define binary mask_
	assert(input_image.channels() * 2 == bounds.cols&& bounds.rows);
	assert(input_image.depth() >= 0 && input_image.depth() <= 6);
	cv::Mat mask_(input_image.rows, input_image.cols, CV_8U, cv::Scalar::all(0));
	mask_.copyTo(output_image);

	// define min/max for proper wrapping
	double max = DBL_MAX;
	double min = -DBL_MAX;

	// loop through threshold bands, seperate into lower & upper thresholds, if any channel has lower > upper,
	// split into 2 bands, else keep original. 
	for (int i = 0; i < bounds.rows; i++) {
		cv::Mat lowerbound(bounds.colRange(0, bounds.cols / 2).rowRange(i, i + 1));
		cv::Mat upperbound(bounds.colRange(bounds.cols / 2, bounds.cols).rowRange(i, i + 1));
		cv::Mat flipedthresh = lowerbound > upperbound;

		if (sum(flipedthresh)[0] > 0) {
			cv::Mat lowerbound2(lowerbound.clone());
			cv::Mat upperbound2(upperbound.clone());

			// use logical array indexing based on flipedthresh
			lowerbound2.setTo(min, flipedthresh);
			upperbound2.setTo(max, flipedthresh);

			cv::inRange(input_image, lowerbound, upperbound2, mask_);
			output_image = output_image | mask_;
			cv::inRange(input_image, lowerbound2, upperbound, mask_);
			output_image = output_image | mask_;
		} else {
			cv::inRange(input_image, lowerbound, upperbound, mask_);
			output_image = output_image | mask_;
		}
	}
}

/** @brief This function performs a channel-wise adaptive threshold to
 *          S and V in an HSV image to find white regions
 *
 *  "output_image","input_image", of type CV_8U (unsigned 8 bit int),
 *  where true == 255 else false == 0
 * 
 *  @param input_image is an image Matrix
 *  @param is a scalar containing the C value for adaptive thresholding
 *  @return output_image is the resulting logical output image with the same
 *          width and height as the input matrix
 */
void FindWhite(const cv::Mat &input_image, const cv::Scalar bounds, int patch_size, cv::Mat &output_image) {
	cv::Mat threshim1, threshim2;
	std::vector<cv::Mat> channels;
	cv::split(input_image, channels);
	threshim1 = channels[1] < bounds[0];
	adaptiveThreshold(channels[2], threshim2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, patch_size, bounds[1]);
	output_image = (threshim1 & threshim2);
}

/** @brief Returns Skeleton image of "Im", where "Im" is a grayscale image,
 *         and output is an 8-bit binary of same dims
 *
 *  This is based on the method propsed on the website of 
 *  Felix Abecassis(http://felix.abecassis.me/2011/09/opencv-morphological-Skeleton/)
 * 
 *  @param input_image is an image Matrix in the HSV colorspace
 *  @return is the resulting logical output image with the same
 *          width and height as the input matrix
 */
cv::Mat Skeletonize(const cv::Mat &image) {
	//define copy of image to apply morphological operators to, and Mats derived from these
	cv::Mat copy;
	image.copyTo(copy);
	cv::Mat skel(copy.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));

	bool done;
	do {
		cv::erode(copy, eroded, element);
		cv::dilate(eroded, temp, element); // temp = open(img)
		cv::subtract(copy, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(copy);

		done = (cv::countNonZero(copy) == 0);
	} while (!done);
	return skel;
}

/** @brief Return binary image of contours in "image",
 *  @param image is an image Matrix
 *  @param min_size is the the minimum size of the contours in the image
 *  @return is the resulting binary image thresholded by min_size
 */
cv::Mat GetContours(const cv::Mat &image, int min_size) {
	//Draw onto a blank image based on found contours
	cv::Mat filtered(image.size(), CV_8UC1, cv::Scalar(0));
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat copy(image.clone());
	findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point());
	cv::Scalar color(255);
	for (size_t i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) >= min_size) {
			drawContours(filtered, contours, i, color, -1, 8, cv::noArray(), 2, cv::Point());
		}
	}
	return filtered;
}