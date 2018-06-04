/**
 * @file traffic_light_detector.cpp
 * @author Adrian Malaran
 * @competition IARRC 2018
 */

// OPENCV includes
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

// OTHER includes
#include <iostream>
#include <vector>
#include <cmath>
#include "traffic_light_detection/traffic_light_detector.hpp"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>

#include <ros/ros.h>

// OpenCv
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

/**@name TrafficLightDetector 
 * @brief Constructor
 */
TrafficLightDetector::TrafficLightDetector(ros::NodeHandle nh) : nh_(nh) {
	ROS_INFO("TrafficLightDetector::TrafficLightDetector");

	Initialize();
}

/**@name Initialize 
 * @brief Invokes necessary functions
 */
void TrafficLightDetector::Initialize() {
	InitializePubsAndSubs();
}

/**@name InitializePubsAndSubs 
 * @brief Initializes the publishers and subscribers of traffic_light_detector
 */
void TrafficLightDetector::InitializePubsAndSubs() {
	image_subscriber_ = nh_.subscribe("/darknet_ros/detection_image", 1,
                            &TrafficLightDetector::ImageReceivedCallback, this);
	bounding_box_subscriber_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1,
							&TrafficLightDetector::BoundingBoxCallback, this);
	traffic_light_image_subscriber_ = nh_.subscribe("/darknet_ros/traffic_light_image", 1, 
							&TrafficLightDetector::TrafficLightImageCallback, this);
}

/**@name TrafficLightImageCallback 
 * @brief Retrieves the cropped traffic light image of the object from YOLO to be parsed
 * @param msg: Message contents of the subscriber
 */
void TrafficLightDetector::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("TrafficLightDetector::TrafficLightImageCallback Recieved Traffic Light Image!");

	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} 
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat traffic_light_image = cv_ptr->image;
	ROS_INFO("Matrix Dimesions - Height: %i Width: %i", traffic_light_image.size().height, traffic_light_image.size().width);
	FindTrafficLightState(traffic_light_image);
}

/**@name FindTrafficLightState
 * @brief Uses OpenCV image processing functions to detect the state of traffic light
 * @param traffic_light_image: The detected traffic light image to be parsed
 */
bool TrafficLightDetector::FindTrafficLightState(const cv::Mat& traffic_light_image) {
	
	cv::Size image_size = traffic_light_image.size();
	int image_height = image_size.height, image_width = image_size.width;


	// Detect Red Light
	cv::Mat red_hsv;
    cvtColor(traffic_light_image, red_hsv, cv::COLOR_BGR2HSV);

	cv::Mat red_mask1, red_mask2;
	cv::inRange(red_hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(red_hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), red_mask2);
	cv::Mat red_mask = red_mask1 | red_mask2;
	int red_pixel_count = cv::countNonZero(red_mask);

	// Detect Green Light
	cv::Mat green_hsv;
    cvtColor(traffic_light_image, green_hsv, cv::COLOR_BGR2HSV);
	cv::Mat green_mask;
    cv::inRange(green_hsv, cv::Scalar((2/12.0) * 255, (2/20.0) * 255,(6.5/10.0) * 255), cv::Scalar((8/12.0) * 255, 255, 255), green_mask);
    int green_pixel_count = cv::countNonZero(green_mask);

	// Make number of pixels as a function of the size of the detected image
	int green_pixel_threshold = (1.0/6) * image_height * image_width;
	int red_pixel_threshold = green_pixel_threshold;

	ROS_INFO("Dimensions Height: %i Width: %i", image_height, image_width);
	ROS_INFO("RED Pixels: %i Threshold: %i", red_pixel_count, red_pixel_threshold);
	ROS_INFO("GRE Pixels: %i Threshold: %i", green_pixel_count, green_pixel_threshold);

    // DETECT RED DROP AND GREEN RISE
    if (red_pixel_count < red_pixel_threshold  && green_pixel_count > green_pixel_threshold) {
    	ROS_INFO("-- GREEN LIGHT --");
    } else if (red_pixel_count > red_pixel_threshold && green_pixel_count < green_pixel_threshold) {
    	ROS_INFO("-- RED LIGHT --");
    } else {
    	ROS_INFO("NO STATE DETECTED");
    }
}

/**@name ImageReceivedCallback
 * @brief Retrieves the entire detected image of the camera
 * @param msg: Message contents of the subscriber
 */
void TrafficLightDetector::ImageReceivedCallback(const sensor_msgs::ImageConstPtr& msg) {
	ROS_DEBUG("TrafficLightDetector::ImageReceivedCallback");
}

void TrafficLightDetector::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
	ROS_DEBUG("TrafficLightDetector::BoundingBoxCallback");
}

/**
 * Tasks Left to do
 * Translates this to the rr_traffic_light_package
 * Import the Yolo Object Detector files to the RobotRacing directory for compilation
 */