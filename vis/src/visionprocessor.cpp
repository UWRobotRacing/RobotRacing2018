#include "visionprocessor.hpp"

#include "vision_fun_2015.hpp"

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>

//OPENCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

Visionprocessor::Visionprocessor(ros::NodeHandle nh_) : it_( nh_ )
{
	
	// Detection Params
	nh_.param<int>("Blob_Size", blobsize, 100);
	nh_.param<int>("Fixed_Threshold", fixedthresh, 3);
	nh_.param<int>("Adaptive_HSV_Max_S", adapt_hsv_S, 20);
	nh_.param<int>("Adaptive_HSV_Min_V", adapt_hsv_V, -40);
	bounds = cv::Scalar (adapt_hsv_S,adapt_hsv_V);
	nh_.param<int>("Adaptive_HSV_MPatch_Size",adapt_hsv_patchsize, 25);
	nh_.param<bool>("PointListOut",point_out, false);

	// Output image stream, binarized and perspective-adjusted
	nh_.param<std::string>("Output_Image_topic", binary_out_im, "/output_video");

	// Output point list
	nh_.param<std::string>("Output_Pointlist_topic", point_vec_out, "/output_point_list");


	//debug mode flag and variables
	bool debug(false);
	nh_.param<bool>("Debug_Flag", debug, false);
	int t;
	
	nh_.param<std::string>("OpenCVMatFile", opencvfilename, "vision_params.yaml");
	cv::FileStorage fs(opencvfilename,cv::FileStorage::READ);
	fs["image_coords"] >> imagecoords;
	fs["world_coords"] >> worldcoords;
	fs["multibounds"] >> multibounds;

	//Get transform output specs
	BEV_size.width = (int)fs["bev_width"];
	fs["bev_height"] >> BEV_size.height;
	fs["bev_resolution"] >> gridresolution;
	fs["grid_offset_x"] >> metaData.origin.position.x;
	fs["grid_offset_y"] >> metaData.origin.position.y;
	fs.release();
	metaData.width = BEV_size.width;
  	metaData.height = BEV_size.height;
  	metaData.resolution = gridresolution;

	transform_matrix = cv::getPerspectiveTransform(imagecoords, worldcoords);

	//Publishers
	image_pub_ = it_.advertise(binary_out_im, 1);
	sobel_pub_ = it_.advertise("sobel_output",1);
	pointList_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(point_vec_out,1);
}


void Visionprocessor::output_params()
{
		//nav_msgs::MapMetaData metaData;
		std::cout	<< "Map Width: " << metaData.width << std::endl
  			  	<< "Map Hieght: " << metaData.height << std::endl
  				<< "Map Resolution: " << metaData.resolution << std::endl		
				<< "Occ Grid Topic Out: " << point_vec_out << std::endl
				<< "Binary Output Topic: " << binary_out_im << std::endl
				<< "Min Blob Size: " << blobsize << std::endl
				<< "Use Fixed Threshold: " << fixedthresh << std::endl
				<< "Adaptive HSV max Saturation: " << adapt_hsv_S << std::endl
				<< "Adaptive HSV V difference from mean: " << adapt_hsv_V << std::endl
				<< "Adaptive HSV PatchSize: " << adapt_hsv_patchsize << std::endl
				<< "Multibounds (fixed bands): " << multibounds << std::endl;
				
		std::cout 	<< "Transformation matrix: " << transform_matrix << std::endl
				<< "Image Coords: " << imagecoords << std::endl
				<< "World Coords: " << worldcoords << std::endl
				<< "Output width: " << BEV_size.width << std::endl
				<< "Output height: " << BEV_size.height << std::endl
				<< "Output Pointlist: " << point_out << std::endl;
}

void Visionprocessor::findwhite_transform(const sensor_msgs::Image::ConstPtr& msg)
{
	
	try
	{
		cv_input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		cv_input_bridge->image.copyTo(im_input);

		cvtColor(im_input, Im1_HSV, CV_BGR2HSV, 3);

		cv::warpPerspective(Im1_HSV, Im1_HSV_warped, transform_matrix, BEV_size,cv::INTER_LINEAR, cv::BORDER_REPLICATE);

		switch (fixedthresh)
		{
			case 0:
				multithresh(Im1_HSV_warped,multibounds,mask_warped1);
				break;
			case 1:
				adapthresh_white_HSV(Im1_HSV_warped,bounds,adapt_hsv_patchsize, mask_warped1);
				break;
			default:
				multithresh(Im1_HSV_warped,multibounds,mask_warped1);
				adapthresh_white_HSV(Im1_HSV_warped,bounds,adapt_hsv_patchsize, mask_warped2);
				cv::bitwise_or(mask_warped1,mask_warped2,mask_warped1);
				break;
		}
		if (!((mask.cols == mask_warped1.cols)&&(mask.rows == mask_warped1.rows)))
		{
			mask = cv::Mat(im_input.rows, im_input.cols, CV_8U, cv::Scalar::all(255));
			cv::warpPerspective(mask, mask, transform_matrix, BEV_size);
		}

		cv::Mat out = filtersize(mask_warped1 & mask, blobsize);

		// find mask
		//Copy to output bridge
		out.copyTo(cv_output_bridge.image);
		cv_output_bridge.encoding = "mono8";

		//Input Image has been processed and published
		image_pub_.publish(cv_output_bridge.toImageMsg());



		//New Functions for sobel line detection
		//cv::Mat sobel_out;
		//Sobel(skeletonize(filtersize(out,2)), sobel_out, CV_8U, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
		//sobel_out = filtersize(sobel_out, 5);
		//sobel_out.copyTo(cv_output_bridge.image);
		//cv_output_bridge.encoding = "mono8";
		//sobel_pub_.publish(cv_output_bridge.toImageMsg());
		
		if(point_out)
		{
			occupancy.clear();
          		occupancy.reserve(out.cols*out.rows);

          		data_pointer = out.data;
          		for(int i_row = 0; i_row < out.rows; i_row++)
          		{
            			for(int i_col = 0; i_col < out.cols; i_col++)
            			{
		      			value1 = *data_pointer;
		      			data_pointer++;

		     			if(value1 == 0)
		      			{
		        			occupancy.push_back(-1);
		      			}
		      			else
		      			{
		        			occupancy.push_back(100);
		      			}
            			}
          		}
			grid_msg.data = occupancy;
			grid_msg.info = metaData;
          	pointList_pub_.publish(grid_msg);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

