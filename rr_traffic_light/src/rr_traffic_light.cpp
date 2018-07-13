/*
 * @file rr_traffic_light
 * @author Adrian Malaran(AdrianMalaran)
 * @author Toni Ogunmade(oluwatoni)
 * @author Jamie Kim
 * @author Jason Leung
 * @competition IARRC 2018
 */

#include "rr_traffic_light.hpp"

/**@name TrafficLightDetector
 * @brief Constructor
 */
TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh) : nh_(nh) {
  ROS_INFO("TrafficLightProcessor::TrafficLightProcessor");

  Initialize();
}

/**@name Initialize
 * @brief Invokes necessary functions
 */
void TrafficLightProcessor::Initialize() {
  InitializePubsAndSubs();

  // Initialize Traffic Light State
  traffic_light_state_ = NO_TRAFFIC_LIGHT;

  // Service
  client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");

  consecutive_green_detection = 0;

  nh_.param<int>("Testmode", test_mode_, 0);
}

/**@name InitializePubsAndSubs
 * @brief Initializes the publishers and subscribers of traffic_light_detector
 */
void TrafficLightProcessor::InitializePubsAndSubs() {
  shutdown_yolo_pub_ = nh_.advertise<std_msgs::Int8>("/darknet_ros/shutdown", 1 , true);

  image_sub_ = nh_.subscribe("/darknet_ros/detection_image", 1,
                            &TrafficLightProcessor::ImageReceivedCallback, this);
  bounding_box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1,
              &TrafficLightProcessor::BoundingBoxCallback, this);
  traffic_light_image_sub_ = nh_.subscribe("/darknet_ros/traffic_light_image", 1,
              &TrafficLightProcessor::TrafficLightImageCallback, this);
}

/**@name TrafficLightImageCallback
 * @brief Retrieves the cropped traffic light image of the object from YOLO to be parsed
 * @param msg: Message contents of the subscriber
 */
void TrafficLightProcessor::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG("TrafficLightProcessor::TrafficLightImageCallback Recieved Traffic Light Image!");

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat traffic_light_image = cv_ptr->image;
  ROS_DEBUG("Matrix Dimesions - Height: %i Width: %i", traffic_light_image.size().height, traffic_light_image.size().width);
  FindTrafficLightColorState(traffic_light_image);
}

/**@name FindTrafficLightState
 * @brief Uses OpenCV image processing functions to detect the state of traffic light
 * @param traffic_light_image: The detected traffic light image to be parsed
 */
bool TrafficLightProcessor::FindTrafficLightColorState(const cv::Mat& traffic_light_image) {
  // Dimensions
  cv::Size image_size = traffic_light_image.size();
  int image_height = image_size.height, image_width = image_size.width;

  // Service Call
  std_srvs::Empty srv;

  if (image_height == 0 || image_width == 0) return false;

  // Detect Red Light
  cv::Mat red_hsv;
  cvtColor(traffic_light_image, red_hsv, cv::COLOR_BGR2HSV);

  cv::Mat red_mask1, red_mask2;
  cv::inRange(red_hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), red_mask1);
  cv::inRange(red_hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), red_mask2);
  cv::Mat red_mask = red_mask1 | red_mask2;
  int red_pixel_count = cv::countNonZero(red_mask);
  // Detect Green Light
  cv::Mat green_hsv;
  cvtColor(traffic_light_image, green_hsv, cv::COLOR_BGR2HSV);
  cv::Mat green_mask;
  cv::inRange(green_hsv, cv::Scalar(42, 25, 166), cv::Scalar(170, 255, 255), green_mask);
  int green_pixel_count = cv::countNonZero(green_mask);

  // Make number of pixels as a function of the size of the detected image
  int green_pixel_threshold = (1.0/9.35) * image_height * image_width;
  int red_pixel_threshold = green_pixel_threshold;

  ROS_DEBUG("Dimensions Height: %i Width: %i", image_height, image_width);
  ROS_INFO("RED Pixels: %i Threshold: %i", red_pixel_count, red_pixel_threshold);
  ROS_INFO("GRE Pixels: %i Threshold: %i", green_pixel_count, green_pixel_threshold);

  // DETECT RED DROP AND GREEN RISE
  if (red_pixel_count < red_pixel_threshold  && green_pixel_count > green_pixel_threshold) {
    ROS_INFO("-- GREEN ---------------------------------------");
    traffic_light_state_ = GREEN;

    // Invoke Service call
    consecutive_green_detection ++;

    if (consecutive_green_detection > 3) { 
      if (client_.call(srv) && !test_mode_)
        ShutdownTrafficLightNodes();
    }
  } else if (red_pixel_count > red_pixel_threshold && green_pixel_count < green_pixel_threshold) {
    ROS_INFO("--------------------------------------- RED --");
    traffic_light_state_ = RED;
    consecutive_green_detection = 0;
  } else {
    ROS_INFO("NONE");
    traffic_light_state_ = NO_TRAFFIC_LIGHT;
  }
}

/**@name GetTrafficLightState
 * @brief Get traffic_light_state_ variable
 */
bool TrafficLightProcessor::GetTrafficLightState() {
  return traffic_light_state_;
}

/**@name ShutdownTrafficLightNodes
 * @brief Publishes a message to Darknet Node to shutdown
 */
void TrafficLightProcessor::ShutdownTrafficLightNodes() {
  ROS_INFO("TrafficLightProcessor::ShutdownTrafficLightNodes Shutting down Darknet Yolo node");
  std_msgs::Int8 shutdown_msg;
  shutdown_yolo_pub_.publish(shutdown_msg);
  ROS_INFO("TrafficLightProcessor::ShutdownTrafficLightNodes Shutting down Traffic Light node");
  ros::shutdown();
}

/**@name ImageReceivedCallback
 * @brief Retrieves the entire detected image of the camera
 * @param msg: Message contents of the subscriber
 */
void TrafficLightProcessor::ImageReceivedCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG("TrafficLightProcessor::ImageReceivedCallback");
}

void TrafficLightProcessor::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
  ROS_DEBUG("TrafficLightProcessor::BoundingBoxCallback");
}
