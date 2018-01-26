// Copyright [2016] University of Waterloo Robotics Team
// Robot Racing Traffic Light Main
// Author: Ji Min Kim, Oluwatoni Ogunmade
// Date: 2016 05 17


// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

// OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// Helper includes
#include "rr_traffic_light.hpp"

#define CAMERA_FRAMERATE 15

int main(int argc, char **argv)
{

  // Initiate trafficLight Node
  ros::init(argc, argv, "trafficLightNode");
  ros::NodeHandle nh_;
  std::cout << "Starting trafficLightNode\n";
  
  std::string camera_source;
  nh_.param<std::string>("Camera_Source_Topic", camera_source, "/traffic_light/image_raw");///traffic_light/usb_cam_traffic_light/image_raw");
  int testmode;
  nh_.param<int>("Testmode", testmode, 0);
  
  image_transport::ImageTransport it_(nh_);
  TrafficLightProcessor tlproc(nh_);
  image_transport::Subscriber image_sub_ = it_.subscribe(camera_source, 1, &TrafficLightProcessor::imageCallback, &tlproc);

  ROS_INFO("Traffic Light Node: Exposure Delay Ended. Ready for Detection.");

  ros::Rate r(CAMERA_FRAMERATE);

  while (ros::ok() && (tlproc.GetTLState() != GREEN || testmode ))
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

