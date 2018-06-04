/**
 * @file main.cpp
 * @author Toni Ogunmade(oluwatoni)
 * @author Ji Min Kim
 * @competition IARRC 2018
 */

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


/**
 * @Requirements
 * TODO (Adrian): 
 * - Refactor main function to depend on darknet_ros node to retrieve the TL state instead of polling
 * - Remove all unnecesarry comments
 * - Integrate service call for traffic light to initiate race
 */

/**
 * @brief initializes and starts the traffic light detection node 
 * @return int 
 */
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
  // image_transport::Subscriber image_sub_ = it_.subscribe(camera_source, 1, &TrafficLightProcessor::imageCallback, &tlproc);

  ROS_INFO("Traffic Light Node: Exposure Delay Ended. Ready for Detection.");

  ros::Rate r(CAMERA_FRAMERATE);

  while (ros::ok() && (tlproc.GetTrafficLightState() != GREEN || testmode ))
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}