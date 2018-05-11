// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Endline Detection Main
// Author: Angela Gu
// Date: 2018 04 18

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "endline_detection.hpp"
#include <std_msgs/Bool.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "endline_detection");
	ROS_INFO("Initializing endline_detection");	
    std::string camera_source = "/robot_racing/forward_facing_cam/image_raw";
    ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
    EndlineCounter ec(nh_);
    image_transport::Subscriber sub_ = it_.subscribe(camera_source, 1, &EndlineCounter::img_callback, &ec);
	ros::spin();
    return 0;
}
