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


void imageCallback (const sensor_msgs::ImageConstPtr& msg) {

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "endline_detection");
	//  "Initializing endline_detection\n\r";
	
    std::string camera_source;
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    //nh.param<std::string>("Camera_Source_Topic", camera_source, "/usb_cam/image_raw");
    // EndlineCounter ec(nh);
    image_transport::Subscriber sub = it.subscribe("camera_source",1, imageCallback);
	
	ros::spin();

    return 0;
}
