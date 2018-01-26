// Copyright [2017] University of Waterloo Robotics Team
// Robot Racing Traffic_Light Node Class Header File
// Author: Jamie Kim, Jason Leung, Oluwatoni Ogunmade
// Date: 2016 03 26

#ifndef RR_TRAFFIC_LIGHT
#define RR_TRAFFIC_LIGHT

//Traffic light states
#define GREEN 1
#define RED 0
#define NO_TRAFFIC_LIGHT -1
#define MULTIPLE_LIGHTS -2

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

// OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


class TrafficLightProcessor
{
    private:
        ros::NodeHandle nh_;

        // Subsribe to an traffic light image 
        std_msgs::Int8 tl_sub_;
        std::string tl_sub_name_;
        image_transport::Subscriber tl_sub_sub_;

        // Publish the traffic light state
        std_msgs::Int8 tl_state;
        std_msgs::Float32 red_value;
        std_msgs::Float32 green_value;
        std::string tl_pub_name_;
        ros::Publisher tl_state_pub;
        ros::Publisher red_pub;
        ros::Publisher green_pub;

        // Other variables
        cv_bridge::CvImagePtr cv_input_bridge;
        cv_bridge::CvImage cv_output_bridge;
        cv::Mat im_input;
    	std::string binary_out_im;
	    image_transport::Publisher image_pub_; 
        std::vector<cv::Mat> images;
        std::vector<double> gPixel, rPixel;
        image_transport::ImageTransport it_;

        int checkback;
        int framenum;
        int curr_vec_index;
        int prev_vec_index;
        bool traffic_light_detected;
        bool red_drop;
        bool green_rise;

        // Threshold params for filtered image
        double red_thresh,green_thresh;
    	int diffThresh;

        double traffic_light_coords[4];

    public:
    	TrafficLightProcessor();
        TrafficLightProcessor(ros::NodeHandle nh);
    	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        bool FindTL();
        int FindTLState();
        int GetTLState();
};

#endif

