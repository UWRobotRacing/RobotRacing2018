#include "vision_fun_2015.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// This function is intended to work in the same fashion as CV::inRange
// but allows for a cv::Matrix of min and max ranges to be passed, with channel wrapping supported
// Inputs: "Im" must be an image matrix, "bounds" is a n-row by m-column matrix, where n is the number
// of threshold bands to be accepted, and m is 2*(number of colour channels in Im), "Im_Log" is the resulting
// logical output image with the same width and hieght as "Im", of type CV_8U (unsigned 8 bit int), 
// where true == 255 else false == 0
void multithresh(const cv::Mat &Im, const cv::Mat &bounds, cv::Mat &Im_Log)
{
	//Confirm Dims and define binary mask
	assert(Im.channels() * 2 == bounds.cols&& bounds.rows);
	assert(Im.depth() >= 0 && Im.depth() <= 6);
	cv::Mat mask(Im.rows, Im.cols, CV_8U, cv::Scalar::all(0));
	mask.copyTo(Im_Log);

	// define min/max for proper wrapping
	double max = DBL_MAX;
	double min = -DBL_MAX;

	// loop through threshold bands, seperate into lower & upper thresholds, if any channel has lower > upper,
	// split into 2 bands, else keep original. 
	for (int i = 0; i < bounds.rows; i++)
	{
		cv::Mat lowerbound(bounds.colRange(0, bounds.cols / 2).rowRange(i, i + 1));
		cv::Mat upperbound(bounds.colRange(bounds.cols / 2, bounds.cols).rowRange(i, i + 1));
		cv::Mat flipedthresh = lowerbound > upperbound;

		if (sum(flipedthresh)[0] > 0)
		{
			cv::Mat lowerbound2(lowerbound.clone());
			cv::Mat upperbound2(upperbound.clone());

			// use logical array indexing based on flipedthresh
			lowerbound2.setTo(min, flipedthresh);
			upperbound2.setTo(max, flipedthresh);

			cv::inRange(Im, lowerbound, upperbound2, mask);
			Im_Log = Im_Log | mask;
			cv::inRange(Im, lowerbound2, upperbound, mask);
			Im_Log = Im_Log | mask;
		}
		else
		{
			cv::inRange(Im, lowerbound, upperbound, mask);
			Im_Log = Im_Log | mask;
		}
	}
}

// This function performs a channel-wise adaptive threshold to S and V in an HSV image to find white regions
// Inputs: "Im" must be an image matrix in HSV, "bounds" is a scalar containing the C value for adaptivethreshold
// for each channel "Im_Log" is the resulting logical output image with the same width and hieght as "Im",
// of type CV_8U (unsigned 8 bit int), where true == 255 else false == 0
void adapthresh_white_HSV(const cv::Mat &Im, const cv::Scalar bounds, int patchsize, cv::Mat &Im_Log)
{

	cv::Mat threshim1, threshim2;
	std::vector<cv::Mat> channels;
	cv::split(Im, channels);
	threshim1 = channels[1] < bounds[0];
	adaptiveThreshold(channels[2], threshim2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, patchsize, bounds[1]);
	Im_Log = (threshim1 & threshim2);
}

// Returns Skeleton image of "Im", where "Im" is a grayscale image, and output is an 8-bit binary of same dims
// This is based on the method propsed on the website of F�lix Abecassis
cv::Mat skeletonize(const cv::Mat &Im)
{
	//define copy of image to apply morphological operators to, and Mats derived from these
	cv::Mat copy;
	Im.copyTo(copy);
	cv::Mat skel(copy.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));

	bool done;
	do
	{
		cv::erode(copy, eroded, element);
		cv::dilate(eroded, temp, element); // temp = open(img)
		cv::subtract(copy, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(copy);

		done = (cv::countNonZero(copy) == 0);
	} while (!done);
	return skel;
}

// Return binary image of blobs in "Im1", thresholded by "minsize" (minimum blob area in pixels)
cv::Mat filtersize(const cv::Mat &Im1, int minsize)
{
	//Draw onto a blank image based on found contours
	cv::Mat filtered(Im1.size(), CV_8UC1, cv::Scalar(0));
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat copy(Im1.clone());
	findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point());
	cv::Scalar color(255);
	for (size_t i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) >= minsize)
		{
			drawContours(filtered, contours, i, color, -1, 8, cv::noArray(), 2, cv::Point());
		}
	}
	return filtered;
}
