/*
 * @file rr_traffic_light
 * @author Toni Ogunmade(oluwatoni)
 * @author Jamie Kim
 * @author Jason Leung
 * @competition IARRC 2018
 */

// OPENCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OTHER includes
#include <iostream>
#include <vector>
#include <cmath>
#include "rr_traffic_light.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <ros/console.h>

// Darknet
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// using namespace cv;

// /**
//  * @brief Construct a new Traffic Light Processor:: Traffic Light Processor object
//  * @param nh_ the nodehandle object
//  */
// TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh_): it_( nh_ )
// { 
//   // Detection Params for Red
//   nh_.param<double>("red_threshold", red_thresh, .85);
//   // Detection Params for Green
//   nh_.param<double>("green_threshold", green_thresh, 20000);
//   nh_.param<int>("CheckbackFrames",checkback,3);
//   images.reserve(checkback);

//   // Output image stream, binarized
//   nh_.param<std::string>("Output_Image_topic", binary_out_im_, "/output_video");


//   // Publishers
//   image_pub_ = it_.advertise(binary_out_im_, 1);
//   tl_pub_name_ = "/enable";
//   tl_state_pub = nh_.advertise<std_msgs::Int8>(tl_pub_name_, 1);
//   red_pub =  nh_.advertise<std_msgs::Float32>("red_test", 1);
//   green_pub =  nh_.advertise<std_msgs::Float32>("green_test", 1);

//   framenum = 0;
// 	cv::Mat temp;
// 	for (int i = 0; i < checkback; i++)
// 	{
// 		images.push_back(temp);
// 		gPixel.push_back(0);
// 		rPixel.push_back(0);
// 	}

//   red_drop = false;
//   green_rise = false;

//   tl_state.data = NO_TRAFFIC_LIGHT;
//   traffic_light_detected = false;
// }

// /**
//  * @brief serves as the callback for each image received from the traffic light
//  * @param msg image message
//  */
// void TrafficLightProcessor::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
// 	if(tl_state.data != GREEN)
//   {
//   //TODO Remove the weird try catch
//     try
//     {    
//       framenum++;
//       printf("%d \n", framenum);
//       curr_vec_index = framenum % checkback;
//       prev_vec_index = (framenum + 1) % checkback;
//       cv_input_bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//       cv_input_bridge_->image.copyTo(images.at(curr_vec_index));

//       if (framenum < checkback)
//       {
//         return;
//       }

//       tl_state.data = FindTLState();
//     }

//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }
//   }
// return;
// }

// /**
//  * @brief returns false 
//  * 
//  * @return true 
//  * @return false 
//  */
// bool TrafficLightProcessor::FindTL()
// {
//   return false;
// }

// /**
//  * @brief finds the state of the traffic light
//  * 
//  * it blurs the image and then converts it to the HSV colorspace and 
//  * thresholds for drop in the green pixels in the image
//  * 
//  * @return true if the traffic light is green false if otherwise 
//  */
// int TrafficLightProcessor::FindTLState()
// {
//   int isGreen = 0;
//   printf("Looking for Traffic!\n");
//   //prep the image with a blur to even it out
//   cv::GaussianBlur(images.at(curr_vec_index), images.at(curr_vec_index), cv::Size(31, 31), 0, 0);

//   cv::Mat hsv;
//   //threshold on the HSV representation of the image
//   cv::cvtColor(images.at(curr_vec_index), hsv, CV_BGR2HSV);
//   cv::Mat red_light, green_light, mask1, mask2;
 
//   // Extract the S channel
//   std::vector<cv::Mat> hsv_planes(3);

//   if (red_drop == false)
//   {
//     //Threshold the HSV image to get only red colors
//     cv::inRange(hsv, cv::Scalar((0/12.0) * 255, (1.7/20.0) * 255,(7/10.0) * 255), cv::Scalar((1/12.0) * 255, 255, 255), mask1);
//     cv::inRange(hsv, cv::Scalar((6/12.0) * 255, (1.7/20.0) * 255,(7/10.0) * 255), cv::Scalar((12/12.0) * 255, 255, 255), mask2);

//     red_light = mask1 | mask2;

//     //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_light).toImageMsg();
//     //image_pub_.publish(msg);

//     // rPixel.at(curr_vec_index) = (cv::mean(red)[0]);
//     rPixel.at(curr_vec_index) = (cv::mean(red_light)[0]);

//     printf("red new: %f old: %f\n", rPixel.at(curr_vec_index), rPixel.at(prev_vec_index));

//     red_value.data = rPixel.at(curr_vec_index);
//     red_pub.publish(red_value);
    
//     if((rPixel.at(curr_vec_index) < (rPixel.at(prev_vec_index) * red_thresh)) && (framenum > (checkback*2))) 
//     {
//        red_drop = true;
//     }
//   }

//   if (green_rise == false)//&& red_drop)
//   {
//     //Threshold the HSV image to get only red colors
//     cv::inRange(hsv, cv::Scalar((2/12.0) * 255, (2/20.0) * 255,(6.5/10.0) * 255), cv::Scalar((8/12.0) * 255, 255, 255), mask1);

//     green_light = mask1;

//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", green_light).toImageMsg();
//     image_pub_.publish(msg);

//     gPixel.at(curr_vec_index) = (cv::sum(green_light)[0]);

//     printf("green new: %f old: %f\n", gPixel.at(curr_vec_index), gPixel.at(prev_vec_index));

//     green_value.data = gPixel.at(curr_vec_index);
//     green_pub.publish(green_value);

//     if(((gPixel.at(curr_vec_index) - gPixel.at(prev_vec_index)) > green_thresh) && red_drop) 
//     {
//        green_rise = true;
//     }
//   }

//   if(red_drop && green_rise) 
//   {
//     isGreen = 1;
//     ROS_INFO("Traffic Light Green %d %d ", red_drop, green_rise);
//   }
//   return isGreen;
// }

// /**
//  * @brief return the state of the traffic light
//  * @return int traffic light state
//  */
// int TrafficLightProcessor::GetTLState()
// {
//   return tl_state.data;
// }


// New Methods


/**@name TrafficLightDetector 
 * @brief Constructor
 */
TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh) : nh_(nh) {
  ROS_INFO("TrafficLightProcessor::TrafficLightProcessor");

  Initialize();
}

/**@name Initialize 
 * @brief Invokes necessary functions
 */
void TrafficLightProcessor::Initialize() {
  InitializePubsAndSubs();

  // Initialize Traffic Light State
  traffic_light_state_ = NO_TRAFFIC_LIGHT;

  // Service
  client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race", true);
}

/**@name InitializePubsAndSubs 
 * @brief Initializes the publishers and subscribers of traffic_light_detector
 */
void TrafficLightProcessor::InitializePubsAndSubs() {
  image_subscriber_ = nh_.subscribe("/darknet_ros/detection_image", 1,
                            &TrafficLightProcessor::ImageReceivedCallback, this);
  bounding_box_subscriber_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1,
              &TrafficLightProcessor::BoundingBoxCallback, this);
  traffic_light_image_subscriber_ = nh_.subscribe("/darknet_ros/traffic_light_image", 1, 
              &TrafficLightProcessor::TrafficLightImageCallback, this);
}

/**@name TrafficLightImageCallback 
 * @brief Retrieves the cropped traffic light image of the object from YOLO to be parsed
 * @param msg: Message contents of the subscriber
 */
void TrafficLightProcessor::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("TrafficLightProcessor::TrafficLightImageCallback Recieved Traffic Light Image!");

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
  FindTrafficLightColorState(traffic_light_image);
}

/**@name FindTrafficLightState
 * @brief Uses OpenCV image processing functions to detect the state of traffic light
 * @param traffic_light_image: The detected traffic light image to be parsed
 */
bool TrafficLightProcessor::FindTrafficLightColorState(const cv::Mat& traffic_light_image) {
  // Dimensions
  cv::Size image_size = traffic_light_image.size();
  int image_height = image_size.height, image_width = image_size.width;
  
  std_srvs::Empty srv;


  if (image_height == 0 || image_width == 0) return false;

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
      traffic_light_state_ = GREEN;

      // Invoke Service call
      client_.call(srv);
    } else if (red_pixel_count > red_pixel_threshold && green_pixel_count < green_pixel_threshold) {
      ROS_INFO("-- RED LIGHT --");
      traffic_light_state_ = RED;
    } else {
      ROS_INFO("NO STATE DETECTED");
      traffic_light_state_ = NO_TRAFFIC_LIGHT;
    }
}

/**@name GetTrafficLightState
 * @brief Get traffic_light_state_ variable
 */
bool TrafficLightProcessor::GetTrafficLightState() {
  return traffic_light_state_;
}

/**@name ImageReceivedCallback
 * @brief Retrieves the entire detected image of the camera
 * @param msg: Message contents of the subscriber
 */
void TrafficLightProcessor::ImageReceivedCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG("TrafficLightProcessor::ImageReceivedCallback");
}

void TrafficLightProcessor::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
  ROS_DEBUG("TrafficLightProcessor::BoundingBoxCallback");
}

/**
 * Tasks Left to do
 * Translates this to the rr_traffic_light_package
 * Import the Yolo Object Detector files to the RobotRacing directory for compilation
 */