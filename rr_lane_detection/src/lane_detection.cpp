/** @file lane_detection.cpp
 *  @author Matthew Post(mpost)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

//ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

//OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//Helper includes
#include "thresholding.hpp"
#include "shadow_removal.hpp"
#include "lane_detection_processor.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "lane_detection");
  std::string camera_source;
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  nh.param<std::string>("Camera_Source_Topic", camera_source, "/usb_cam/image_raw");
  ROS_INFO("Starting lane detection");
  lane_detection_processor lane_detection_proc(nh);
  image_transport::Subscriber image_sub_ = 
    it.subscribe(camera_source, 1, &lane_detection_processor::FindLanes, &lane_detection_proc);
  lane_detection_proc.PrintParams();

  ros::spin();
  return 0;
}