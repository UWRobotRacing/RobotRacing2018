/*
 * Line_Node.cpp
 *
 *  Created on: Mar 15, 2015
 *      Author: mpost
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
#include "vision_fun_2015.hpp"
#include "visionprocessor.hpp"

int main(int argc, char** argv)
{
	//Node and image transport initialization
	ros::init(argc, argv, "lineNode");
	ros::NodeHandle nh_;
	std::cout << "Starting Up lineNode\n";

	std::string camera_source;
	nh_.param<std::string>("Camera_Source_Topic", camera_source, "/usb_cam/image_raw");

	image_transport::ImageTransport it_(nh_);
	Visionprocessor visproc(nh_);
	image_transport::Subscriber image_sub_ = it_.subscribe(camera_source, 1, &Visionprocessor::findwhite_transform, &visproc);
	
	visproc.output_params();
	
	while(ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}


