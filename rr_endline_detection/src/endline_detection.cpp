#include "endline_detection.hpp"

//constructor
EndlineCounter::EndlineCounter (ros::NodeHandle nh_) : it_(nh_) 
{
	//pub_ = nh_.advertise<std_msgs::Bool>("endline",1);
	status = 0;
	client_ = nh_.serviceClient<std_srvs::Trigger>("supervisor", true);
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
		
		blur_img = mag_img.clone();
		cv::GaussianBlur(mag_img, blur_img, cv::Size(7,7), 0, 0);

		//detect endline
		status = status << 1;
		if (blob_detector(blur_img)){
			++status;
			ROS_INFO("sts %x", status);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	// Update GUI Window
	cv::imshow("OPENCV_WINDOW", blur_img);
	cv::waitKey(3);

	//notify service if consecutive 1's
	status &= 0xF;
	ROS_INFO("2sts %x", status);
	if (status > 8) {
		if(client_.call(srv))
		{
			ROS_INFO("TRUE");
		} else {
			ROS_INFO("FALSE");
		}
	}
}

//determines by area if blob is large enough
 bool EndlineCounter::blob_detector(cv::Mat img) {
	cv::SimpleBlobDetector::Params params;
	params.filterByArea = true;
	params.minArea = 50;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(img, keypoints);

	if (keypoints.size() > 0) {
 		return true;
	}
	
	return false;
 }