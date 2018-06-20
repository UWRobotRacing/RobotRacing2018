/**
 * @file rr_traffic_light.hpp
 * @brief Traffic Light Header File
 * @author Jamie Kim
 * @author Jason Leung
 * @author Toni Ogunmade(oluwatoni)
 * @author Adrian Malaran
 * @competition IARRC 2018
 */

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
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class TrafficLightProcessor
{
    private:
    
    ros::NodeHandle nh_;

    ros::ServiceClient client_;

    ros::Subscriber image_sub_;
    ros::Subscriber found_object_sub_;
    ros::Subscriber bounding_box_sub_;
    ros::Subscriber traffic_light_image_sub_;

    int traffic_light_state_;

    public:

    TrafficLightProcessor();
    TrafficLightProcessor(ros::NodeHandle nh);

    void Initialize();
    void InitializePubsAndSubs();

    bool FindTrafficLightColorState(const cv::Mat& trafic_light_image);
    bool GetTrafficLightState();

    void TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ImageReceivedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg);

    void TrafficLightDetectedCallback(const sensor_msgs::ImageConstPtr& image, 
    const darknet_ros_msgs::BoundingBoxes& bounding_box);
};

#endif

