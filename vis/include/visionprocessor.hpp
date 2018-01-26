#ifndef __VISIONPROC
#define __VISIONPROC

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>

//OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


class Visionprocessor {
	private:
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		image_transport::Publisher sobel_pub_;
		ros::Publisher pointList_pub_;
		cv_bridge::CvImagePtr cv_input_bridge;
		cv_bridge::CvImage cv_output_bridge;
		cv::Mat im_input;
		image_transport::ImageTransport it_;

		nav_msgs::MapMetaData metaData;
		nav_msgs::OccupancyGrid grid_msg;

		
		std::string point_vec_out;
		std::string binary_out_im;

  		//Pointer to mat data
  		uchar* data_pointer;
  		uchar value1;

  		//Vector for msg
  		std::vector<int8_t> occupancy;

		//various params
		int blobsize;
		int fixedthresh;
		int adapt_hsv_S;
		int adapt_hsv_V;
		int adapt_hsv_patchsize;
		cv::Scalar bounds;
		cv::Mat multibounds;
		cv::Mat Im1_HSV, Im1_HSV_warped, mask_warped1,mask_warped2, mask;
		
		std::string opencvfilename;
		cv::Mat imagecoords;
		cv::Mat worldcoords;
		
		cv::Mat transform_matrix;
		cv::Size BEV_size;
		double gridresolution;


		// I/O behavioural params
		bool point_out;
	public:
		Visionprocessor();
		Visionprocessor(ros::NodeHandle nh);
		void output_params();
		void findwhite_transform(const sensor_msgs::Image::ConstPtr& msg);
};

	
#endif

