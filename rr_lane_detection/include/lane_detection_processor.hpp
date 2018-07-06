/** @file lane_detection_processor.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef __LANE_DETECTION_PROCESSOR_HPP
#define __LANE_DETECTION_PROCESSOR_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

//OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

class lane_detection_processor
{
  private:
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher sobel_pub_;
    ros::Publisher pointList_pub_;
    cv_bridge::CvImagePtr cv_input_bridge_;
    cv_bridge::CvImage cv_output_bridge_;
    cv::Mat im_input_;
    image_transport::ImageTransport it_;

    nav_msgs::MapMetaData meta_data_;
    nav_msgs::OccupancyGrid grid_msg_;

    std::string point_vec_out_;
    std::string binary_out_im_;

    //Pointer to mat data
    uchar* data_pointer_;
    uchar value1_;

    //Vector for msg
    std::vector<int8_t> occupancy_;

    //various params
    int blob_size_;
    int fixed_threshold_;
    int adapt_hsv_S_;
    int adapt_hsv_V_;
    int adapt_hsv_patch_size_;
    cv::Scalar bounds_;
    cv::Mat multibounds_;
    cv::Mat Im1_Shadows_Removed, Im1_HSV_, Im1_HSV_warped_, mask_warped_1_,mask_warped_2_, mask_;
    
    std::string opencv_file_name_;
    cv::Mat image_coords_;
    cv::Mat world_coords_;
    cv::Mat transform_matrix_;
    cv::Size BEV_size_;
    double grid_resolution_;

    bool simulation_;
    // I/O behavioural params
    bool point_out_;
  public:
    lane_detection_processor(ros::NodeHandle nh);
    void PrintParams();
    void FindLanes(const sensor_msgs::Image::ConstPtr& msg);
};

#endif //__LANE_DETECTION_PROCESSOR_HPP