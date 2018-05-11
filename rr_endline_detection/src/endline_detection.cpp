#include "endline_detection.hpp"

EndlineCounter::EndlineCounter (ros::NodeHandle nh_) : it_(nh_) 
{
	pub_ = nh_.advertise<std_msgs::Bool>("endline",1);
	state = BEGINNING;
}

void EndlineCounter::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("%d", state);
	cv::Mat init_img, hsv_img, mag_img, mag_img2;
	cv_bridge::CvImagePtr cv_ptr;
 	int iLowH = 145;
 	int iHighH = 165;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		//filter for magenta
		init_img = cv_ptr->image;
		cv::cvtColor(init_img,hsv_img, CV_BGR2HSV);
		cv::inRange(hsv_img, cv::Scalar(iLowH, 0,0), cv::Scalar(iHighH,255,255), mag_img);
		
		mag_img2 = mag_img.clone();
		cv::GaussianBlur(mag_img, mag_img2, cv::Size(7,7), 0, 0);

		//detect line


	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	// Update GUI Window
	cv::imshow("OPENCV_WINDOW", mag_img2);
	cv::waitKey(3);

	//publish 
	// if(state & PAST_END) {
	// 	pub.publish(true);
	// } else {
	// 	pub.publish(false);
	// }
}

void EndlineCounter::find_el_state(){
	/*if (numofBlobs > 0)
        if (currstate == linestate.START)
            currstate = linestate.STARTL;
        elseif (currstate == linestate.MIDDLE)
            currstate = linestate.ENDL;
        end
    else
        if (currstate == linestate.STARTL)
            currstate = linestate.MIDDLE;
        elseif (currstate == linestate.ENDL)
            currstate = linestate.FINISH;
        end
    end */
}

 bool EndlineCounter::blob_detector() {
	cv::SimpleBlobDetector *detector = new SimpleBlobDetector();
	detector->params.filterByArea = true;
	detector->params.minArea = 500;

 	return true;
 }