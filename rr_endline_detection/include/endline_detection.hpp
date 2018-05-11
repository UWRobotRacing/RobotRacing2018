#ifndef ENDLINE_DETECTION_HPP
#define ENDLINE_DETECTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

enum states {
	BEGINNING = 0x00,
	START_LINE = 0x03,
	MIDDLE = 0x04,
	END_LINE = 0x07,
	PAST_END = 0x08
};

class EndlineCounter {
	private :
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		
		cv_bridge::CvImagePtr imgptr;
		cv::Mat img;
		
		ros::Publisher pub_;
	
	public :
		int state;
		int get_state();

		EndlineCounter(ros::NodeHandle nodeh);
		void img_callback(const sensor_msgs::ImageConstPtr&);
		void find_el_state();
		bool blob_detector();
};

#endif /*ENDLINE_DETECTION_HPP*/