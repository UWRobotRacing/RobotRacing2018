#include "endline_detection.hpp"

EndlineCounter::EndlineCounter (ros::NodeHandle nh) : it(nh) 
{
	pub = nh.advertise<std_msgs::Bool>("endline",1);
	state = BEGINNING;
}

void EndlineCounter::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("hello world!");
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	{
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}	
	// Update GUI Window
	cv::imshow("OPENCV_WINDOW", cv_ptr->image);
	cv::waitKey(3);

	//publish 
	// if(state & PAST_END) {
	// 	pub.publish(true);
	// } else {
	// 	pub.publish(false);
	// }
}

void EndlineCounter::find_el_state(){}