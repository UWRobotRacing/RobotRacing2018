/** @file endline_detection.cpp
 * @brief Robot Racer endline detection function implementation
 *
 * Topics Subscribed:
 *   /rr_vehicle/front_facing_cam/image_raw
 *
 * Service called:
 *   /Supervisor/count_lap         --used by supervisor node
 *
 * @author Angela Gu (angegu)
 * @author Toni Ogunmade (oluwatoni)
 */

#include "endline_detection.hpp"

//constructor
EndlineCounter::EndlineCounter (ros::NodeHandle nh_) : it_(nh_) 
{
  detection_status_ = false;
  hysteresis_counter_ = 0;
  hysteresis_constant_ = 2;
  client_ = nh_.serviceClient<std_srvs::Trigger>("/Supervisor/count_lap");
}

//callback to handle detection
void EndlineCounter::ImgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat init_img, hsv_img, mag_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  int iLowH = 145;
  int iHighH = 165;
  std_srvs::Trigger srv;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //filter for magenta
    init_img = cv_ptr->image;
    cv::cvtColor(init_img, hsv_img, CV_BGR2HSV);
    cv::inRange(hsv_img, cv::Scalar(iLowH, 0,0), cv::Scalar(iHighH,255,255), mag_img);
    cv::GaussianBlur(mag_img, mag_img, cv::Size(7,7), 0, 0);

    //calls BlobDetector to evaluate area
    if (BlobDetector(mag_img))
    {
      if (!detection_status_)
      {
        //increment when endline not confirmed
        hysteresis_counter_++;
        if (hysteresis_counter_ > hysteresis_constant_)
        {
          //counter has passed threshold constant
          hysteresis_counter_ = 0;
          detection_status_ = true;
          ROS_INFO("DETECTED");
        }
      }
      else
      {
        //decay if detection not true
        if (hysteresis_counter_)
        {
          hysteresis_counter_--;
        }
      }
    }
    else
    {
      //BlobCounter did not detect anything
      if (!detection_status_)
      {
        //decay
        if (hysteresis_counter_)
        {
          hysteresis_counter_--;
        }
      }
      else
      {
        //post detection, detect when endline no longer in sight
        hysteresis_counter_++;
        if (hysteresis_counter_ > hysteresis_constant_){
          hysteresis_counter_ = 0;
          detection_status_ = false;
          ROS_INFO("NO LONGER DETECTED");

          //make service call
          if (client_.call(srv))
          {
            if (srv.response.success)
            {
              ROS_INFO("SUCCESS");
              ros::shutdown();
            }
          }
        }
      }
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

//determines by area if blob is large enough
bool EndlineCounter::BlobDetector(cv::Mat img)
{
  cv::SimpleBlobDetector::Params params;
  params.filterByArea = true;
  params.filterByCircularity = true;
  params.filterByColor = false;
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.minArea = 250;
  params.maxCircularity = 0.4;
  params.minCircularity = 0;

  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(img, keypoints);

  if (keypoints.size() > 0)
  {
    return true;
  }
  return false;
}
