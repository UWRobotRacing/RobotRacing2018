/**
 * @file traffic_light_detector.hpp
 * @brief Traffic Light Header File
 * @author Adrian Malaran
 * @competition IARRC 2018
 */

#ifndef RR_TRAFFIC_LIGHT_DETECTOR_H
#define RR_TRAFFIC_LIGHT_DETECTOR_H

// ROS includes
#include <ros/ros.h>

// OPENCV includes
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class TrafficLightDetector
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber image_subscriber_;
    ros::Subscriber found_object_subscriber_;
    ros::Subscriber bounding_box_subscriber_;
    ros::Subscriber traffic_light_image_subscriber_;

public:
    TrafficLightDetector();
    TrafficLightDetector(ros::NodeHandle nh);

    void Initialize();
    void InitializePubsAndSubs();

    bool FindTrafficLightState(const cv::Mat& trafic_light_image);

    void TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ImageReceivedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg);

    void TrafficLightDetectedCallback(const sensor_msgs::ImageConstPtr& image, 
    const darknet_ros_msgs::BoundingBoxes& bounding_box);

};

#endif

