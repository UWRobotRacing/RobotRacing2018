#ifndef ENDLINE_DETECTION_HPP
#define ENDLINE_DETECTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>

class EndlineCounter {
  private :
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    ros::ServiceClient client_;
    bool detection_status_;
    int hysteresis_counter_;
    int hysteresis_constant_;
  
  public :
    EndlineCounter(ros::NodeHandle);
    void img_callback(const sensor_msgs::ImageConstPtr&);
    bool blob_detector(cv::Mat);
};

#endif /*ENDLINE_DETECTION_HPP*/