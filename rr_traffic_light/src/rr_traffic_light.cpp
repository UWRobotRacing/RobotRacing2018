// Copyright [2016] University of Waterloo Robotics Team
// Robot Racing Traffic_Light Node Class Header File
// Author: Jamie Kim, Jason Leung
// Date: 2016 05 17


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

using namespace cv;

TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh_): it_( nh_ )
{ 
  // Detection Params for Red
  nh_.param<double>("red_threshold", red_thresh, .85);
  // Detection Params for Green
  nh_.param<double>("green_threshold", green_thresh, 20000);
  nh_.param<int>("CheckbackFrames",checkback,3);
  images.reserve(checkback);

  // Output image stream, binarized
  nh_.param<std::string>("Output_Image_topic", binary_out_im, "/output_video");


  // Publishers
  image_pub_ = it_.advertise(binary_out_im, 1);
  tl_pub_name_ = "/enable";
  tl_state_pub = nh_.advertise<std_msgs::Int8>(tl_pub_name_, 1);
  red_pub =  nh_.advertise<std_msgs::Float32>("red_test", 1);
  green_pub =  nh_.advertise<std_msgs::Float32>("green_test", 1);

  framenum = 0;
	cv::Mat temp;
	for (int i = 0; i < checkback; i++)
	{
		images.push_back(temp);
		gPixel.push_back(0);
		rPixel.push_back(0);
	}

  red_drop = false;
  green_rise = false;

  tl_state.data = NO_TRAFFIC_LIGHT;
  traffic_light_detected = false;
}

void TrafficLightProcessor::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	if(tl_state.data != GREEN)
  {
  //TODO Remove the weird try catch
    try
    {    
      framenum++;
      printf("%d \n", framenum);
      curr_vec_index = framenum % checkback;
      prev_vec_index = (framenum + 1) % checkback;
      cv_input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_input_bridge->image.copyTo(images.at(curr_vec_index));

      if (framenum < checkback)
      {
        return;
      }

      tl_state.data = FindTLState();
    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
return;
}

//looks for a red light in an image an uses that to describe a traffic light
bool TrafficLightProcessor::FindTL()
{
  return false;
}

int TrafficLightProcessor::FindTLState()
{
  int isGreen = 0;
  printf("Looking for Traffic!\n");
  //prep the image with a blur to even it out
  cv::GaussianBlur(images.at(curr_vec_index), images.at(curr_vec_index), cv::Size(31, 31), 0, 0);

  cv::Mat hsv;
  //threshold on the HSV representation of the image
  cv::cvtColor(images.at(curr_vec_index), hsv, CV_BGR2HSV);
  cv::Mat red_light, green_light, mask1, mask2;
 
  // Extract the S channel
  std::vector<cv::Mat> hsv_planes(3);

  if (red_drop == false)
  {
    //Threshold the HSV image to get only red colors
    cv::inRange(hsv, cv::Scalar((0/12.0) * 255, (1.7/20.0) * 255,(7/10.0) * 255), cv::Scalar((1/12.0) * 255, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar((6/12.0) * 255, (1.7/20.0) * 255,(7/10.0) * 255), cv::Scalar((12/12.0) * 255, 255, 255), mask2);

    red_light = mask1 | mask2;

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_light).toImageMsg();
    //image_pub_.publish(msg);

  //  rPixel.at(curr_vec_index) = (cv::mean(red)[0]);
    rPixel.at(curr_vec_index) = (cv::mean(red_light)[0]);

    printf("red new: %f old: %f\n", rPixel.at(curr_vec_index), rPixel.at(prev_vec_index));

    red_value.data = rPixel.at(curr_vec_index);
    red_pub.publish(red_value);
    
    if((rPixel.at(curr_vec_index) < (rPixel.at(prev_vec_index) * red_thresh)) && (framenum > (checkback*2))) 
    {
       red_drop = true;
    }
  }

  if (green_rise == false)//&& red_drop)
  {
    //Threshold the HSV image to get only red colors
    cv::inRange(hsv, cv::Scalar((2/12.0) * 255, (2/20.0) * 255,(6.5/10.0) * 255), cv::Scalar((8/12.0) * 255, 255, 255), mask1);

    green_light = mask1;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", green_light).toImageMsg();
    image_pub_.publish(msg);

    gPixel.at(curr_vec_index) = (cv::sum(green_light)[0]);

    printf("green new: %f old: %f\n", gPixel.at(curr_vec_index), gPixel.at(prev_vec_index));

    green_value.data = gPixel.at(curr_vec_index);
    green_pub.publish(green_value);

    if(((gPixel.at(curr_vec_index) - gPixel.at(prev_vec_index)) > green_thresh) && red_drop) 
    {
       green_rise = true;
    }
  }

  if(red_drop && green_rise) 
  {
    isGreen = 1;
    ROS_INFO("Traffic Light Green %d %d ", red_drop, green_rise);
  }
  return isGreen;

}

int TrafficLightProcessor::GetTLState()
{
  return tl_state.data;
}
