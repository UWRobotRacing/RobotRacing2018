/**
 * @file main.cpp
 * @author Adrian Malatan
 * @competition IARRC 2018
 */

#include "traffic_light_detection/traffic_light_detector.hpp"
// ROS includes
#include <ros/ros.h>

/**
 * @brief initializes and starts the traffic light detection node 
 * @return int 
 */
int main(int argc, char **argv)
{
  // Initiate trafficLight Node
  ros::init(argc, argv, "traffic_light_detector");
  ros::NodeHandle nh_;
  std::cout << "Starting traffic_light_detector\n";

  TrafficLightDetector traffic_light_detector(nh_);

  ros::Rate r(15);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}