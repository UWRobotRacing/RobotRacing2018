// Sample for XIMEA Software Package V2.57
#include <m3api/xiApi.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace std;

#define HandleResult(res,place) if (res!=XI_OK) {ROS_ERROR_STREAM("Error after " << place << " (" << res << ")"); if (xiH){ xiCloseDevice(xiH); xiH = NULL;} goto reattempt;}

void custom_copy(char * input, char * till, std::vector<unsigned char>::iterator dest)
{
  while (input != till)
  {
		*dest = *input;
		input ++;
		dest++;
	}
}

XI_RETURN setImageDataFormat(const HANDLE &xiH, const string& image_format,
		 string &encoding, int &bpp){
   int format = 0;
   if (image_format == string("XI_MONO16")){
	format = XI_MONO16;
	encoding = string("mono16");
	bpp = 2;
   }
   else if (image_format == string("XI_RGB24")){
	format = XI_RGB24;
	encoding = string("bgr8");
	bpp = 3;
   }
   else if (image_format == string("XI_RGB32")){
	format = XI_RGB32;
	encoding = string("bgr16");
	bpp = 3;
   }
   else if (image_format == string("XI_RGB_PLANAR")){
	format = XI_RGB_PLANAR;
        ROS_WARN_STREAM( "This is unsupported in ROS" );
	return -1; 
   }
   else if (image_format == string("XI_RAW8")){
	format = XI_RAW8;
	encoding = string("mono8");
	bpp = 1;
	
   }
   else if (image_format == string("XI_RAW16")){
	format = XI_RAW16;
	encoding = string("mono16");
	bpp = 2;
   }
   else {
	format = XI_MONO8;
	encoding = string("mono8");
	bpp = 1;
   }
   return xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, format);
};

using namespace std;

int main(int argc, char ** argv)
{
	// initialization stuff
	HANDLE xiH = NULL; XI_RETURN stat = XI_OK;
	XI_IMG image;
	image.size = sizeof(XI_IMG);
	image.bp = NULL;
	image.bp_size = 0;
	int time_us = 1000;
	
	//syncing paramters
	ros::Time currTime;
  int secs, nsecs, msecs;
  double frame_period;

	//ros initialization
	ros::init(argc, argv, "ximea_camera");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  image_transport::ImageTransport it(pnh);	

  int cams_on_bus;
  string cam_name_;
  string yaml_url_;
  string device_sn_string;
  string image_data_format;
  int desired_frame_rate;  
  int desired_reconnect_rate = 1;
  bool auto_exposure;
  int exposure_time;
  int bandwidth_safety_margin;
  bool enable_binning;
  int downsample_factor;

  string ros_image_data_encoding;
  int ros_image_bpp;

  int rect_height, rect_left, rect_top, rect_width;
    
  pnh.param<string>("camera_name", cam_name_, "camera");
  pnh.param<string>("info_url", yaml_url_, "");
  pnh.param<string>("serial_no", device_sn_string, "");
  pnh.param<int>("cams_on_bus", cams_on_bus, 4);
  pnh.param<int>("frame_rate", desired_frame_rate, 30);
  pnh.param<bool>("auto_exposure", auto_exposure, true);
  pnh.param<int>("exposure_time", exposure_time, 1000);
  pnh.param<int>("bandwidth_safety_margin", bandwidth_safety_margin, 30);
  pnh.param<bool>("binning", enable_binning, false);
  pnh.param<int>("downsample_scale", downsample_factor, 1);
  pnh.param<string>("image_data_format", image_data_format, "XI_MONO8");
  
  //region of interest parameters
  pnh.param<int>("rect_left", rect_left, 0);
  pnh.param<int>("rect_top", rect_top, 0);
  pnh.param<int>("rect_width", rect_width, 1280);
  pnh.param<int>("rect_height", rect_height, 1024);

  ROS_INFO_STREAM("Using Camera with Serial No. " << device_sn_string);
    
	//camera info manager
	camera_info_manager::CameraInfoManager cam_info_manager_(pnh, cam_name_);
	cam_info_manager_.loadCameraInfo(yaml_url_);

	image_transport::Publisher ros_cam_pub = it.advertise(string("image_raw"), 1);
	ros::Publisher cam_info_pub = pnh.advertise<sensor_msgs::CameraInfo>( string("camera_info"), 1);

	sensor_msgs::Image ros_image_;
	sensor_msgs::CameraInfo cam_info_;

	int cam_buffer_size_;
	char* cam_buffer_;

	xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);

	int interface_data_rate = 2400; // when USB3 hub is used
	// calculate datarate for each camera
	int camera_data_rate = interface_data_rate / cams_on_bus;
	// each camera should send less data to keep transfer reliable
	camera_data_rate -= (int)(camera_data_rate*(bandwidth_safety_margin/100.0));
	// set data rate

  ros::Rate inner_loop_rate(10 *desired_frame_rate);
  ros::Rate outer_loop_rate(desired_reconnect_rate);

  while(ros::ok())
  {

    if(device_sn_string.empty())
    {
      stat = xiOpenDevice(0, &xiH);
      HandleResult(stat,"xiOpenDevice");
    }
    else 
    {
      stat = xiOpenDeviceBy(XI_OPEN_BY_SN, device_sn_string.c_str(), &xiH);
      HandleResult(stat,"xiOpenDeviceBy");
    }

    if( auto_exposure )
    {
      stat = xiSetParamFloat(xiH, XI_PRM_AEAG, 1);
            HandleResult(stat,"xiOSetParamFloat (Auto Exposure)");
    }
    else
    {
      stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, exposure_time);
            HandleResult(stat,"xiOSetParamInt (Exposure Time)");
    }

    stat = xiSetParamInt(xiH, XI_PRM_LIMIT_BANDWIDTH , camera_data_rate);
          HandleResult(stat,"xiSetParamInt (Camera Data Rate)");

    stat = setImageDataFormat(xiH, image_data_format, ros_image_data_encoding, ros_image_bpp);
          HandleResult(stat,"xiSetParamInt (Camera Data Rate)");
	  
    
   
    
    if( enable_binning )
    {
      stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, downsample_factor);
            HandleResult(stat,"xiSetParamInt (Downsampling)");      
    }
    
    stat = xiSetParamInt(xiH, XI_PRM_WIDTH, rect_width);
          HandleResult(stat,"xiSetParamInt (aoi width)");
    stat = xiSetParamInt(xiH, XI_PRM_HEIGHT, rect_height);
          HandleResult(stat,"xiSetParamInt (aoi height)");

    stat = xiSetParamInt(xiH, XI_PRM_OFFSET_X, rect_left);
          HandleResult(stat,"xiSetParamInt (aoi left)");
    stat = xiSetParamInt(xiH, XI_PRM_OFFSET_Y, rect_top);
          HandleResult(stat,"xiSetParamInt (aoi top)");
    
    // Start acquisition
    stat = xiStartAcquisition(xiH);
    HandleResult(stat,"xiStartAcquisition");

    // frame_period in seconds
    frame_period = 1.0 / desired_frame_rate;
    while (ros::ok() && xiH != NULL)
    {	
      static ros::Time last_image_time(0);
      ros::Time current_time = ros::Time::now();

      double dt = (current_time - last_image_time).toSec();

      if (dt >= frame_period)
      {
        //syncing at the millisecond rate
        stat = xiGetImage(xiH, 1000, &image);
        HandleResult(stat,"xiGetImage");

        // getting image from camera
        cam_buffer_ = (char *)image.bp;
        cam_buffer_size_ = image.width * image.height * ros_image_bpp;

        ros_image_.data.resize(cam_buffer_size_);
        ros_image_.encoding = ros_image_data_encoding; 
        ros_image_.width = image.width;
        ros_image_.height = image.height;
        ros_image_.step = image.width*ros_image_bpp;

        copy((char*) cam_buffer_,
            ((char*) cam_buffer_) + cam_buffer_size_,
            ros_image_.data.begin());
        
        ros::Time now = ros::Time::now();
        ros_image_.header.stamp = now;
        
        ros_cam_pub.publish(ros_image_);

        cam_info_ = cam_info_manager_.getCameraInfo();
        cam_info_.header.stamp = now;
        cam_info_pub.publish(cam_info_);
        
        last_image_time = current_time;
      }
      
      ros::spinOnce();
      inner_loop_rate.sleep();
      
      // Close device
    }
    
reattempt: 
    if (xiH) 
      xiCloseDevice(xiH);
    
    ROS_WARN_STREAM( "Attempting to RECONNECT to camera!" );
    outer_loop_rate.sleep();   
  }
  
  if (xiH) 
    xiCloseDevice(xiH);
  
}
