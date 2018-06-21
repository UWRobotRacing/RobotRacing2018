/** @file main.cpp
 * @brief Robot Racer endline detection main method
 *
 * This function subscribes to camera feed and then its only task is to call supervisor when endline is detected.
 *  
 *
 * @author Angela Gu (angegu)
 * @author Toni Ogunmade (oluwatoni)
 */

#include <ros/ros.h>
#include "endline_detection.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "endline_detection");
  ROS_INFO("Initializing endline_detection");	
  std::string camera_source = "/rr_vehicle/front_facing_cam/image_raw";
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  EndlineCounter ec(nh_);
  image_transport::Subscriber sub_ = it_.subscribe(camera_source, 1, &EndlineCounter::ImgCb, &ec);
  ros::spin();
  return 0;
}
