#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include "robot_racing_cv.hpp"

#ifndef PI
#define PI 3.14159265
#endif

//typedefs
typedef std::vector<cv::Point2i> point_vector_i;

//GLOBALS
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
cv_bridge::CvImagePtr cv_input_bridge;
cv_bridge::CvImage cv_output_bridge;
bool enable_flag = false;
bool new_input_image = false;
cv::Mat rgb_input;
cv::Mat camera_Matrix;
cv::Mat dist_Coeffs;
int RANSAC_LEVELS = 5;
int ITER_PER_LEVEL = 20;
int MAX_BLOBS_TO_CONSIDER_LINE = 10;
double LINE_FITTING_DISTANCE_THRESHOLD = 2;

void enableMessageCb(const std_msgs::Bool &enable_msg)
{
  if (enable_msg.data == true)
  {
    enable_flag = true;
  }
  else
  {
    enable_flag = false;
  }
}

void findLinesRansac(const std::vector < point_vector_i > blobs, 
                     std::vector < point_vector_i > &line_blobs)
{
  //Vectors to be used
  point_vector_i bright_blob; //holds union of MAX_BLOBS_TO_CONSIDER_LINE vectors, NOTE: blobs are mutually exclusive
  point_vector_i current_line; //holds current line fitting consensus set
  std::vector< point_vector_i> line_candidates; //holds line fitting sets for current level of iteration 
  line_blobs.clear(); // Line vectors to be returned

  //RANSAC point data
  int idx_A = 0;
  int idx_B = 0;
  cv::Point2i line_point_A;
  cv::Point2i line_point_B;  

  //Take up to 3 biggest blobs and merge them into a single larger blob  
  bright_blob.clear();
  bright_blob = blobs.back(); //Blob is sorted in ascending order
  int blobs_to_add = std::min(int(blobs.size()), MAX_BLOBS_TO_CONSIDER_LINE);
  int blobs_added = 1;

  cv::Point2i candidate_point;
  double line_A_value;
  double line_B_value;
  double line_C_value;
  double distance_denominator;
  double candidate_distance;

  while(blobs_added < blobs_to_add)
  {
    point_vector_i next_blob;
    next_blob = blobs.at(blobs.size() - 1 - blobs_added);

    //Insert next blob
    bright_blob.insert(bright_blob.end(), next_blob.begin(), next_blob.end());
    blobs_added++;
  }

  for(int ransac_level = 0; (ransac_level < RANSAC_LEVELS) && (bright_blob.size() > 2); ransac_level++)
  {
    line_candidates.clear();
    for(int ransac_iteration = 0; ransac_iteration < ITER_PER_LEVEL; ransac_iteration++)
    {
      current_line.clear();
      //pick two random indices
      idx_A = 0;
      idx_B = 0;
      while(idx_A == idx_B)
      {
        idx_A = std::rand() % bright_blob.size();
        idx_B = std::rand() % bright_blob.size();
      }
      line_point_A = bright_blob.at(idx_A);
      line_point_B = bright_blob.at(idx_B);

      //determine equation of line: ax + by + c = 0, with A = (x0,y0) and B = (x1,y1)
      line_A_value = line_point_B.y - line_point_A.y;
      line_B_value = line_point_A.x - line_point_B.x;
      line_C_value = line_point_A.y * line_point_B.x - line_point_A.x * line_point_B.y;

      //Distance will be determined by: abs(Ax+By+C)/sqrt(A^2+B^2)
      distance_denominator = std::sqrt(line_A_value*line_A_value + line_B_value*line_B_value);

      //Determine size of consistency set!
      for(point_vector_i::iterator point_iterator = bright_blob.begin(); point_iterator != bright_blob.end(); ++point_iterator)
      {
        candidate_point = *point_iterator;
        candidate_distance = std::abs(line_A_value * candidate_point.x 
                                      + line_B_value * candidate_point.y 
                                      + line_C_value) / distance_denominator;
        if (candidate_distance < LINE_FITTING_DISTANCE_THRESHOLD)
        {
          current_line.push_back(*point_iterator);
        }
      }//end for: check every point for consistency with the candidate line

      //Add this candidate to the set of potential candidates
      line_candidates.push_back(current_line);

    }//end for: for each iteration of RANSAC on this level

    //Sort line candidates by consensus size (ascending order)
    std::sort(line_candidates.begin(), line_candidates.end(), compare_blobs);

    //Final line candidate now has the largest consensus set
    line_blobs.push_back(line_candidates.back());

    //Remove selected consensus set from sample set
    removeScatteredSubvector(bright_blob,line_candidates.back());
    //Next level of line fitting!
  }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);    
    cv_input_bridge->image.copyTo(rgb_input);

    //Correct image distortion ######
    //cv::undistort(cv_input_bridge->image,rgb_input, camera_Matrix, dist_Coeffs);

    new_input_image = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "circuit_race_cv");
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh("~");
  image_transport::ImageTransport it_(nh_);

  //Subscribers
  image_sub_ = it_.subscribe("/RRCamera/image_raw", 1, imageCb);
  ros::Subscriber enable_sub = nh_.subscribe("/RR1/Enable", 1, enableMessageCb);

  //Publishers
  image_pub_ = it_.advertise("/circuit_race_cv/output_video", 1);
  ros::Publisher traffic_signal_detector_pub = nh_.advertise<std_msgs::Bool>("circuit_race_cv/traffic_light_go",1);
  ros::Publisher lane_occupancy_grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("lane_map",1);

  //Parameters (specify in launch file)
  int grayscale_threshold_value;
  private_nh.param<int>("threshold_value",grayscale_threshold_value,200);
  bool debug_flag;
  private_nh.param<bool>("debug_flag",debug_flag,false);
  bool output_video_flag;
  private_nh.param<bool>("output_video_flag",output_video_flag,true);
  double max_eccentricity;
  private_nh.param<double>("max_eccentricity",max_eccentricity,0.15);
  double red_signal_shift_ratio;
  private_nh.param<double>("red_signal_shift_ratio",red_signal_shift_ratio,0.9);  
  double green_signal_shift_ratio;
  private_nh.param<double>("green_signal_shift_ratio",green_signal_shift_ratio,1.1);
  //Parameters already declared as globals
  private_nh.param<int>("max_blobs_to_consider_line", MAX_BLOBS_TO_CONSIDER_LINE, 3);
  private_nh.param<int>("ransac_levels", RANSAC_LEVELS, 5);
  private_nh.param<int>("ransac_iter_per_level", ITER_PER_LEVEL, 20);
  private_nh.param<double>("line_fitting_distance_threshold", LINE_FITTING_DISTANCE_THRESHOLD, 2);
  int LINE_FIT_MINIMUM_PIXELS;
  private_nh.param<int>("line_fit_minimum_lane_pixels", LINE_FIT_MINIMUM_PIXELS, 200);

  cv_bridge::CvImage debug_im_bridge_1;
  cv_bridge::CvImage debug_im_bridge_2;
  cv_bridge::CvImage debug_im_bridge_3;
  cv_bridge::CvImage debug_im_bridge_4;
  image_transport::Publisher debug_image_pub_1;
  image_transport::Publisher debug_image_pub_2;
  image_transport::Publisher debug_image_pub_3;
  image_transport::Publisher debug_image_pub_4;

  if(debug_flag == true)
  {
  debug_image_pub_1 = it_.advertise("/circuit_race_cv/debug_video1", 1);
  debug_image_pub_2 = it_.advertise("/circuit_race_cv/debug_video2", 1);
  debug_image_pub_3 = it_.advertise("/circuit_race_cv/debug_video3", 1);
  debug_image_pub_4 = it_.advertise("/circuit_race_cv/debug_video4", 1);
  }

  //Output messages
  std_msgs::Bool traffic_light_go;
  geometry_msgs::Pose2D lane_line_msg;
  geometry_msgs::Pose2D stop_line_msg;

  nav_msgs::MapMetaData metaData;
  metaData.width = 100;
  metaData.height = 100;
  metaData.resolution = 0.05;
  nav_msgs::OccupancyGrid lane_grid_msg;
  lane_grid_msg.info = metaData;

  //Pointer to output mat
  uchar *imagerowPtr;

  //Pointer to mat data
  uchar* pointer_of_tears;
  uchar value_of_tears;

  //Vector for msg
  std::vector<int8_t> occupancy(10000);

  cv::Mat fuuuuuuuuuu;

  //Create polygon to mask the image region where the car obstructs the view
  cv::Point2i car_points[1][27];
  car_points[0][0] = cv::Point(217, 1024);
  car_points[0][1] = cv::Point(229, 959);
  car_points[0][2] = cv::Point(245, 905);
  car_points[0][3] = cv::Point(287, 877);
  car_points[0][4] = cv::Point(326, 880);
  car_points[0][5] = cv::Point(384, 896);
  car_points[0][6] = cv::Point(406, 909);
  car_points[0][7] = cv::Point(434, 860);
  car_points[0][8] = cv::Point(426, 850);
  car_points[0][9] = cv::Point(486, 758);
  car_points[0][10] = cv::Point(505, 745);
  car_points[0][11] = cv::Point(532, 740);// 737);
  car_points[0][12] = cv::Point(572, 739);// 735);
  car_points[0][13] = cv::Point(648, 741);// 741); //MID
  car_points[0][14] =cv::Point(732, 739);// 736);
  car_points[0][15] =cv::Point(814, 738);
  car_points[0][16] =cv::Point(860, 751);
  car_points[0][17] =cv::Point(876, 769);
  car_points[0][18] =cv::Point(928, 835);
  car_points[0][19] =cv::Point(919, 858);
  car_points[0][20] =cv::Point(953, 861);
  car_points[0][21] =cv::Point(964, 878);
  car_points[0][22] =cv::Point(1035, 866);
  car_points[0][23] =cv::Point(1086, 872);
  car_points[0][24] =cv::Point(1113, 897);
  car_points[0][25] =cv::Point(1135, 958);
  car_points[0][26] =cv::Point(1149, 1024);
  const cv::Point2i* car_points_to_draw[1] = {car_points[0]};
  int num_car_points_to_draw[] = {27};

  //Image to Bird's Eye View (BEV) homography matrix
  double H_R1_C1, H_R1_C2, H_R1_C3, H_R2_C1, H_R2_C2, H_R2_C3, H_R3_C1, H_R3_C2, H_R3_C3;
  H_R1_C1 = 1.9097e-3;
  H_R1_C2 = 1.2435e-4;
  H_R1_C3 = -1.3507e0;
  H_R2_C1 = 6.1269e-5;
  H_R2_C2 = -1.9560e-3;
  H_R2_C3 = 1.9667e0;
  H_R3_C1 = 5.3042e-8;
  H_R3_C2 = 5.6057e-6;
  H_R3_C3 = -2.9218e-3;
  cv::Mat H_Camera_to_BEV = (cv::Mat_<double>(3,3) << H_R1_C1, H_R1_C2, H_R1_C3, H_R2_C1, H_R2_C2, H_R2_C3, H_R3_C1, H_R3_C2, H_R3_C3);

  //BEV to image homography matrix
  double H_INV_R1_C1, H_INV_R1_C2, H_INV_R1_C3, H_INV_R2_C1, H_INV_R2_C2, H_INV_R2_C3, H_INV_R3_C1, H_INV_R3_C2, H_INV_R3_C3;
  H_INV_R1_C1 = 4.958349e2;
  H_INV_R1_C2 = 6.731177e2;
  H_INV_R1_C3 = 2.238754e5;
  H_INV_R2_C1 = -2.645765e1;
  H_INV_R2_C2 = 5.143405e2;
  H_INV_R2_C3 = 3.584466e5;
  H_INV_R3_C1 = -4.176047e-2;
  H_INV_R3_C2 = 9.990367e-1;
  H_INV_R3_C3 = 3.495249e2;
  cv::Mat H_INV_Camera_to_BEV = (cv::Mat_<double>(3,3) << H_INV_R1_C1, H_INV_R1_C2, H_INV_R1_C3, H_INV_R2_C1, H_INV_R2_C2, H_INV_R2_C3, H_INV_R3_C1, H_INV_R3_C2, H_INV_R3_C3);

  //Camera Matrix
  camera_Matrix = (cv::Mat_<double>(3,3) << 523.5378, 0, 651.8150, 0, 526.2258, 507.9504, 0, 0, 1);
  //Distortion Coefficients: [k1,k2,p1,p2,k3,k4,k5,k6]
  dist_Coeffs = (cv::Mat_<double>(8,1) << -288.3487e-3, -76.4080e-3, 0, 0, 8.6091e-3, 0, 0, 0);

  //Image variables
  cv::Mat rgb_working;
  cv::Mat gray_input;
  cv::Mat gray_working;
  cv::Mat threshold_mask;
  cv::Mat hsv_working;
  cv::Size full_image_size;
  uint8_t NUM_IMAGE_ROWS;
  cv::Mat output_image;

  //Traffic signal detection variables
  //Signal detections
  bool red_light_turned_off = false;
  bool green_light_turned_on = false;
  //HSV colour bounds
  cv::Scalar lower_bound_red_upper_band(165,75,100);
  cv::Scalar upper_bound_red_upper_band(179,255,255);
  cv::Scalar lower_bound_red_lower_band(0,75,100);
  cv::Scalar upper_bound_red_lower_band(33,255,255);
  cv::Scalar lower_bound_green(45,100,100);
  cv::Scalar upper_bound_green(95,255,255);  
  //HSV colour masks
  cv::Mat hsv_red_mask_lower_band;
  cv::Mat hsv_red_mask_upper_band;
  cv::Mat hsv_green_mask;
  cv::Mat hsv_red_mask;  
  //colour channel Mats
  std::vector<cv::Mat> rgb_channels(3);
  std::vector<cv::Mat> red_and_green_filtered_channels(3);

  //Colour Blobs
  std::vector< point_vector_i > red_blobs;
  std::vector< point_vector_i > green_blobs;
  point_vector_i red_circle_blob;
  point_vector_i green_circle_blob;
  //Object masks
  cv::Mat red_circle_mask;
  cv::Mat green_circle_mask;
  //Object images in mono
  cv::Mat red_circle_image;
  cv::Mat green_circle_image;
  bool red_circle_prev_detected = false;
  bool green_circle_prev_detected = false;

  //Vector of vectors used for connected component analysis
  std::vector < point_vector_i > blobs;
  std::vector < point_vector_i > ransac_lines;
  std::vector < point_vector_i > lane_line_blob;
  std::vector < point_vector_i > stop_line_blob;
  std::vector < point_vector_i > red_circle_blob_vector;
  std::vector < point_vector_i > green_circle_blob_vector;

  //Contours to draw for debug
  std::vector < point_vector_i > contours_to_draw;

  //Lane interpretation variables
  int lane_line_idx = -1;
  int stop_line_idx = -1;
  double lane_heading;
  double lateral_distance_mm;
  double stop_line_distance_mm;

  //Lane display masks
  cv::Mat lane_line_mask;
  cv::Mat stop_line_mask;

  //Variables for BEV mapping & scaling
  std::vector<cv::Point2f> current_BEV;
  std::vector<cv::Point2f> current_image_BEV;
  std::vector<cv::Point2f> desired_BEV;
  current_BEV.clear();
  current_image_BEV.clear();
  desired_BEV.clear();
  current_BEV.push_back(cv::Point2f(-2500,0)); //Bottom left
  current_BEV.push_back(cv::Point2f(-2500,5000)); //Top left
  current_BEV.push_back(cv::Point2f(2500,0)); //Bottom right
  current_BEV.push_back(cv::Point2f(2500,5000)); //Top right
  desired_BEV.push_back(cv::Point2f(0,0)); //Bottom left
  desired_BEV.push_back(cv::Point2f(0,250)); //Top left
  desired_BEV.push_back(cv::Point2f(250,0)); //Bottom right
  desired_BEV.push_back(cv::Point2f(250,250)); //Top right

  //BEV spatial variables
  double BEV_resolution = 20; //In mm
  cv::Size BEV_size(250,250);

  ros::Time tic = ros::Time::now();
  ros::Time toc = ros::Time::now();
  ros::Duration tictoc;

  cv::Mat transform_matrix;
  cv::perspectiveTransform(current_BEV,current_image_BEV,H_INV_Camera_to_BEV);      
  transform_matrix = cv::getPerspectiveTransform(current_image_BEV,desired_BEV);

  //Initialize neural network parameter paths as package path
  std::string net_tx_1_path = ros::package::getPath("robot_racing_2014_cv");
  std::string net_tx_2_path = ros::package::getPath("robot_racing_2014_cv");

  net_tx_1_path.append("/src/net_tx_1.bin");
  net_tx_2_path.append("/src/net_tx_2.bin");
  cv::Size net_tx1_size = cv::Size(46,75);
  cv::Size net_tx2_size = cv::Size(76,2);
  cv::Size patch_size = cv::Size(15,3);

  NeuralNetLaneClassifier lane_detector(net_tx_1_path, net_tx_2_path, net_tx1_size, net_tx2_size, patch_size);



  while(ros::ok())
  {
    if (new_input_image == true) //Input to be processed!
    {
      //Crop out car from view
      cv::fillPoly(rgb_input, car_points_to_draw, num_car_points_to_draw, 1, cv::Scalar(0,0,0), 8);

      contours_to_draw.clear();

      if (enable_flag == false) //Detect "GO!" From Traffic Signal
      {
        //RGB is format of interest
        full_image_size = rgb_input.size();

        //Copy to working variable
        rgb_input.copyTo(rgb_working);
        cv::cvtColor(rgb_working,hsv_working,CV_RGB2HSV);

        //Red Light Detection
        /////////////////////////////////////////////
        //Colour thresholds
        cv::inRange(hsv_working, lower_bound_red_lower_band, upper_bound_red_lower_band, hsv_red_mask_lower_band);
        cv::inRange(hsv_working, lower_bound_red_upper_band, upper_bound_red_lower_band, hsv_red_mask_upper_band);

        //Combine masks
        hsv_red_mask = hsv_red_mask_lower_band + hsv_red_mask_upper_band;
        //Apply morphological "opening" filter
        cv::morphologyEx(hsv_red_mask,hsv_red_mask, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        //Debug: Output morph image
        if(debug_flag == true)
        {
          hsv_red_mask.copyTo(debug_im_bridge_1.image);
          debug_im_bridge_1.encoding = "mono8";
          debug_image_pub_1.publish(debug_im_bridge_1.toImageMsg());
        }

        //Search for red blobs
        red_blobs.clear();
        red_circle_blob.clear();
        cv::Canny(hsv_red_mask,hsv_red_mask, 35, 90);

        //Debug: Output morph image
        if(debug_flag == true)
        {
          hsv_red_mask.copyTo(debug_im_bridge_2.image);
          debug_im_bridge_2.encoding = "mono8";
          debug_image_pub_2.publish(debug_im_bridge_2.toImageMsg());
        }

        findContours(hsv_red_mask, red_blobs, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); // ##### findBlobs apparently needs a binary 0 or 1 image...        

        if (!red_blobs.empty())
        {//Search for red circle blob
          findCircleBlob(red_blobs, red_circle_blob, max_eccentricity);
        }

        if (!red_circle_blob.empty())
        {//Red circle detected
          if (red_circle_prev_detected == false) //Initial detection
          {
            std::cout << "Red light turned on!" << std::endl;
            red_light_turned_off = false;            
          }
          red_circle_prev_detected = true;
        }
        else
        {//No red circle detected
          if(red_circle_prev_detected == true)
          {//Red light was on, now it is off
            std::cout << "Red light turned off!" << std::endl;
            red_light_turned_off = true;
          }
          red_circle_prev_detected = false;
        }
        //////////////////////////////////////////////

        //Green Light Detection
        //////////////////////////////////////////////
        //Colour thresholds
        cv::inRange(hsv_working, lower_bound_green, upper_bound_green, hsv_green_mask);

        cv::morphologyEx(hsv_green_mask,hsv_green_mask, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        //Search for green blobs
        green_blobs.clear();
        green_circle_blob.clear();
        cv::Canny(hsv_green_mask, hsv_green_mask, 35,90);
        findContours(hsv_green_mask, green_blobs, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        
        if (!green_blobs.empty())
        {//Search for green circle blob
          findCircleBlob(green_blobs, green_circle_blob, max_eccentricity);            
        }

        if (!green_circle_blob.empty())
        {//Green circle detected           
          if (green_circle_prev_detected == false) //Initial detection
          {
            std::cout << "Green light turned on!" << std::endl;
            green_light_turned_on = true;            
          }
          green_circle_prev_detected = true;        
        }
        else
        {//No green circle detected
          if(green_circle_prev_detected == true)
          {//green light was on, now it is off
            std::cout << "Green light turned off!" << std::endl;
            green_light_turned_on = false;            
          }
          green_circle_prev_detected = false;
        }        

        //Publish traffic signal status
        if (red_light_turned_off && green_light_turned_on)
        {
          //Go signal has been detected
          traffic_light_go.data = true;          
        }
        else
        {
          traffic_light_go.data = false;
        }
        traffic_signal_detector_pub.publish(traffic_light_go);

        if (debug_flag == true)  
        {
          //Push detected blobs into contour vector
          if(!red_circle_blob.empty())
          {
            contours_to_draw.push_back(red_circle_blob);
          }
          if(!green_circle_blob.empty())
          {
            contours_to_draw.push_back(green_circle_blob);
          }
          //Split RGB into channels
          split(rgb_working,rgb_channels);

          //Mask the channels: Red and Green
          cv::bitwise_and(rgb_channels[0],hsv_green_mask + hsv_red_mask,red_and_green_filtered_channels[0]);
          cv::bitwise_and(rgb_channels[1],hsv_green_mask + hsv_red_mask,red_and_green_filtered_channels[1]);
          cv::bitwise_and(rgb_channels[2],hsv_green_mask + hsv_red_mask,red_and_green_filtered_channels[2]);

          //Merge into output
          cv::merge(red_and_green_filtered_channels,cv_output_bridge.image);

          cv::drawContours(cv_output_bridge.image, contours_to_draw, -1, cv::Scalar(255,255,255), CV_FILLED);

          //Copy to output bridge
          cv_output_bridge.encoding = cv_input_bridge->encoding;

          //Publish
          image_pub_.publish(cv_output_bridge.toImageMsg());
        }

      }//if: Traffic Signal Detection
      else //Go! => Detect the lanes
      {
        //Convert RGB to MONO8
        cv::warpPerspective(rgb_input,rgb_working,transform_matrix,BEV_size);
        cv::cvtColor(rgb_working,gray_input,CV_RGB2GRAY);
        full_image_size = gray_input.size();

        //Copy to working variable
        gray_input.copyTo(gray_working);

        //Debug: Output gray_input
        if(debug_flag == true)
        {
          gray_input.copyTo(debug_im_bridge_1.image);
          debug_im_bridge_1.encoding = "mono8";
          cv::flip(debug_im_bridge_1.image,debug_im_bridge_1.image,0);
          debug_image_pub_1.publish(debug_im_bridge_1.toImageMsg());
        }
        //debug: Output rgb input
        if(debug_flag == true)
        {
          rgb_working.copyTo(debug_im_bridge_4.image);
          debug_im_bridge_4.encoding = "rgb8";
          cv::flip(debug_im_bridge_4.image,debug_im_bridge_4.image,0);
          debug_image_pub_4.publish(debug_im_bridge_4.toImageMsg());
        }        

        //Adaptive Threshold
        cv::adaptiveThreshold(gray_working,threshold_mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 9,-10);

        //Debug: Output adaptive threshold mask
        if(debug_flag == true)
        {
          threshold_mask.copyTo(debug_im_bridge_2.image);
          debug_im_bridge_2.encoding = "mono8";
          cv::flip(debug_im_bridge_2.image,debug_im_bridge_2.image,0);
          debug_image_pub_2.publish(debug_im_bridge_2.toImageMsg());
        }

        lane_detector.classify_image(gray_input,threshold_mask,gray_working);

        //Debug: Output classifier results
        if(debug_flag == true)
        {
          gray_working.copyTo(debug_im_bridge_3.image);
          debug_im_bridge_3.encoding = "mono8";
          cv::flip(debug_im_bridge_3.image,debug_im_bridge_3.image,0);
          debug_image_pub_3.publish(debug_im_bridge_3.toImageMsg());
        }

        //Blob Analysis
        //Note: blobs is sorted, blobs.back() is the most populous blob!
        findBlobs(gray_working/255,blobs);

        if(!blobs.empty())
        {
          findLinesRansac(blobs, ransac_lines);          

          //cv::cvtColor(gray_working,cv_output_bridge.image,CV_GRAY2RGB);
          cv_output_bridge.image = cv::Mat::zeros(gray_working.size(), CV_8U);

          //Iterate over ransac_lines and draw them if they are large enough
          for(auto line_iterator = ransac_lines.begin(); line_iterator != ransac_lines.end(); line_iterator++)
          {
            if(line_iterator->size() > LINE_FIT_MINIMUM_PIXELS)
            {
              lane_line_blob.clear();
              lane_line_blob.push_back(*line_iterator);
              cv::drawContours(cv_output_bridge.image, lane_line_blob, -1, cv::Scalar(255), CV_FILLED);
            }            
          }

          //Resize image to 100x100
          cv::resize(cv_output_bridge.image,lane_line_mask, cv::Size(100,100));
          cv_output_bridge.image = lane_line_mask;
          cv_output_bridge.image.copyTo(fuuuuuuuuuu);

          // std::cout << fuuuuuuuuuu.type() << std::endl;
          // std::cout << fuuuuuuuuuu << std::endl;

          //Convert to occupancy grid
          occupancy.clear();
          occupancy.reserve(10000);

          pointer_of_tears = fuuuuuuuuuu.data;
          for(int i_row = 0; i_row < 100; i_row++)
          {
            for(int i_col = 0; i_col < 100; i_col++)
            {
              value_of_tears = *pointer_of_tears;
              pointer_of_tears++;
              
              if(value_of_tears == 0)
              {
                occupancy.push_back(-1);
              }
              else
              {
                occupancy.push_back(100);
                //std::cout << "here" << std::endl;
              }
            }
          }

          //Publish occupancy!
          lane_grid_msg.data = occupancy;
          lane_occupancy_grid_pub.publish(lane_grid_msg);

          cv_output_bridge.encoding = "mono8";
          //Publish
          cv::flip(cv_output_bridge.image,cv_output_bridge.image,0);
          image_pub_.publish(cv_output_bridge.toImageMsg());

        }//If there are white blobs        
        else
        {
          ROS_ERROR("No lane markings detected");
        }    
      }//If enable = true => detect lanes

      //Input Image has been processed and published
      new_input_image = false;
    }

    ros::spinOnce();
}

  return 0;
}