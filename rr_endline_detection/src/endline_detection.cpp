#include "endline_detection.hpp"

//constructor
EndlineCounter::EndlineCounter (ros::NodeHandle nh_) : it_(nh_) 
{
  detection_status_ = false;
  hysteresis_counter_ = 0;
  hysteresis_constant_ = 2;
  client_ = nh_.serviceClient<std_srvs::Trigger>("/Supervisor/count_lap", true);
}

//callback to handle detection
void EndlineCounter::img_callback(const sensor_msgs::ImageConstPtr& msg)
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

    //detect endline
    if (blob_detector(mag_img))
    {
      if (!detection_status_)
      {
        hysteresis_counter_++;
        if (hysteresis_counter_ > hysteresis_constant_)
        {
          hysteresis_counter_ = 0;
          detection_status_ = true;
          ROS_INFO("DETECTED");
          if(client_.call(srv))
          {
            if (srv.response.success)
            {
              ROS_INFO("SUCCESS");
            }
          }
        }
      }
      else
      {
        if (hysteresis_counter_)
        {
          hysteresis_counter_--;
        }
      }
    }
    else
    {
      if (!detection_status_)
      {
        if (hysteresis_counter_)
        {
          hysteresis_counter_--;
        }
      }
      else
      {
        hysteresis_counter_++;
        if (hysteresis_counter_ > hysteresis_constant_){
          hysteresis_counter_ = 0;
          detection_status_ = false;
          ROS_INFO("NO LONGER DETECTED");
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
bool EndlineCounter::blob_detector(cv::Mat img)
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
