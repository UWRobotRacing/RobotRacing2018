/** @file lane_detection_processor.cpp
 *  @author Matthew Post(mpost)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#include "lane_detection_processor.hpp"
#include "thresholding.hpp"

/** @brief sets the rosparams and obtains the perspective transforms
 *  @param nh is the node handle of the node
 *  @return NONE
 */
lane_detection_processor::lane_detection_processor(ros::NodeHandle nh) : it_(nh)
{
  // Detection Params
  nh.param<int>("Blob_Size", blob_size_, 100);
  nh.param<int>("Fixed_Threshold", fixed_threshold_, 0);
  nh.param<int>("Adaptive_HSV_Max_S", adapt_hsv_S_, 20);
  nh.param<int>("Adaptive_HSV_Min_V", adapt_hsv_V_, -40);
  bounds_ = cv::Scalar(adapt_hsv_S_, adapt_hsv_V_);
  nh.param<int>("Adaptive_HSV_MPatch_Size", adapt_hsv_patch_size_, 25);
  nh.param<bool>("PointListOut", point_out_, true);

  // Output image stream, binarized and perspective-adjusted
  nh.param<std::string>("Output_Image_topic", binary_out_im_, "/output_video");

  // Output point list
  nh.param<std::string>("Output_Pointlist_topic", point_vec_out_, "/output_point_list");

  //debug mode flag and variables
  bool debug(false);
  nh.param<bool>("Debug_Flag", debug, false);

  nh.param<std::string>("OpenCVMatFile", opencv_file_name_, "vision_params.yaml");
  cv::FileStorage fs(opencv_file_name_, cv::FileStorage::READ);
  fs["image_coords"] >> image_coords_;
  fs["world_coords"] >> world_coords_;
  fs["multibounds"] >> multibounds_;

  //Get transform output specs
  BEV_size_.width = (int)fs["bev_width"];
  fs["bev_height"] >> BEV_size_.height;
  fs["bev_resolution"] >> grid_resolution_;
  fs["grid_offset_x"] >> meta_data_.origin.position.x;
  fs["grid_offset_y"] >> meta_data_.origin.position.y;
  fs.release();
  meta_data_.width = BEV_size_.width;
  meta_data_.height = BEV_size_.height;
  meta_data_.resolution = grid_resolution_;

  transform_matrix_ = cv::getPerspectiveTransform(image_coords_, world_coords_);

  //Publishers
  image_pub_ = it_.advertise(binary_out_im_, 1);
  sobel_pub_ = it_.advertise("sobel_output", 1);
  pointList_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(point_vec_out_, 1);
}

/** @brief Prints the parameters
 *  @return NONE
 */
void lane_detection_processor::PrintParams()
{
  //nav_msgs::MapMetaData meta_data_;
  std::cout << "Map Width: " << meta_data_.width << "\n"
     << "Map Hieght: " << meta_data_.height << "\n"
     << "Map Resolution: " << meta_data_.resolution << std::endl
     << "Occ Grid Topic Out: " << point_vec_out_ << std::endl
     << "Binary Output Topic: " << binary_out_im_ << std::endl
     << "Min Blob Size: " << blob_size_ << std::endl
     << "Use Fixed Threshold: " << fixed_threshold_ << std::endl
     << "Adaptive HSV max Saturation: " << adapt_hsv_S_ << std::endl
     << "Adaptive HSV V difference from mean: " << adapt_hsv_V_ << std::endl
     << "Adaptive HSV PatchSize: " << adapt_hsv_patch_size_ << std::endl
     << "Multibounds (fixed bands): " << multibounds_ << std::endl;

  std::cout << "Transformation matrix: " << transform_matrix_ << std::endl
     << "Image Coords: " << image_coords_ << std::endl
     << "World Coords: " << world_coords_ << std::endl
     << "Output width: " << BEV_size_.width << std::endl
     << "Output height: " << BEV_size_.height << std::endl
     << "Output Pointlist: " << point_out_ << std::endl;
}

/** @brief Serves as the subscriber to the lane detection camera
 *         it thresholds for the yellow and white lines
 * 
 *  different thresholding options are avialable
 * 
 *  @param msg is a pointer to the image from the camera
 *  @return publishes an occupancy_ grid with the lines detected as
 *          obstacles
 */
void lane_detection_processor::FindLanes(const sensor_msgs::Image::ConstPtr &msg)
{
  try
  {
    cv_input_bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_input_bridge_->image.copyTo(im_input_);

    cvtColor(im_input_, Im1_HSV_, CV_BGR2HSV, 3);
    cv::warpPerspective(Im1_HSV_, Im1_HSV_warped_, transform_matrix_, BEV_size_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    Multithreshold(Im1_HSV_warped_, multibounds_, mask_warped_1_);
    FindWhite(Im1_HSV_warped_, bounds_, adapt_hsv_patch_size_, mask_warped_2_);
    cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);

    // sets up the BEV mask for that camera to remove everything outside of them
    // masked area
    if (!((mask_.cols == mask_warped_1_.cols) && (mask_.rows == mask_warped_1_.rows)))
    {
      mask_ = cv::Mat(im_input_.rows, im_input_.cols, CV_8U, cv::Scalar::all(255));
      cv::warpPerspective(mask_, mask_, transform_matrix_, BEV_size_);
    }

    cv::Mat out = GetContours(mask_warped_1_ &mask_, blob_size_);
    //cv::Mat out;               // dst must be a different Mat
    //cv::flip(src, out, 1);   

    //find mask_
    //Copy to output bridge
    out.copyTo(cv_output_bridge_.image);
    cv_output_bridge_.encoding = "mono8";

    //Input Image has been processed and published
    image_pub_.publish(cv_output_bridge_.toImageMsg());

    if (point_out_)
    {
      occupancy_.clear();
      occupancy_.reserve(out.cols * out.rows);

      data_pointer_ = out.data;
      for (int i = 0; i < out.rows * out.cols; i++)
      {
        value1_ = *data_pointer_;
        data_pointer_++;
        if (value1_ == 0)
        {
          occupancy_.push_back(-1);
        }
        else
        {
          occupancy_.push_back(100);
        }
      }
      grid_msg_.data = occupancy_;
      grid_msg_.info = meta_data_;
      pointList_pub_.publish(grid_msg_);
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
