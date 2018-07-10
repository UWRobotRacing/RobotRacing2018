/** @file laser_mapper.cpp
 *  @author Jungwook Lee
 *  @author Andrew Jin (DongJunJin)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

// CPP
#include <algorithm>

// Local
#include "laser_mapper.hpp"
#include <occupancy_grid_utils.hpp>

/**
 * @name LaserMapper
 * @brief initiliazes the LaserMapper class
 * @return NONE
 */
LaserMapper::LaserMapper() {
  // Load Parameters
  GetParam();

  // PUBLISHERS
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_name_, 1);
  
  // SUBSCRIBERS
  scan_sub_ = nh_.subscribe(laser_scan_name_, 1, &LaserMapper::LidarCallback, this);
  lane_detection_left_sub_ = nh_.subscribe("/output_point_list_left_cam", 1, &LaserMapper::DetectLeftLaneCallback, this);
  lane_detection_right_sub_ = nh_.subscribe("/output_point_list_right_cam", 1, &LaserMapper::DetectRightLaneCallback, this);

  // Initialize an occupancy grid (std::vector<int>)
  InitMap();
}

/**
 * @name ~LaserMapper
 * @brief destructs the LaserMapper class
 * @return NONE
 */
LaserMapper::~LaserMapper() {
  belief_map_.clear();
}

/**
 * @name InitMap
 * @brief initiliazes the belief_map_ with UNKNOWNS
 * @return NONE
 */
void LaserMapper::InitMap() {
  //Fills the map with Unknown values in the vector
  belief_map_.resize(map_width_*map_height_, UNKNOWN_);
  ROS_INFO("Map Initialized with UNKNOWN, Height: %d, Width: %d", map_height_, map_width_);
}

/**
 * @name GetParam
 * @brief Obtains ROS Params
 * @return NONE
 */
void LaserMapper::GetParam() {
  nh_.param<std::string>("LaserMapper/Occupancy_Grid_Name", occupancy_grid_name_, "/map");
  nh_.param<std::string>("LaserMapper/Laser_Scan_Name", laser_scan_name_, "/scan");

  nh_.param<double>("LaserMapper/map_res", map_res_, 1);
  nh_.param<int>("LaserMapper/map_W", map_width_, 800);
  nh_.param<int>("LaserMapper/map_H", map_height_, 300);
  nh_.param<double>("LaserMapper/LASER_ORIENTATION", LASER_ORIENTATION_, -1);
  nh_.param<int>("LaserMapper/INFLATE_OBS", inflate_obstacle_, 2);

  nh_.param<double>("LaserMapper/max_angle", max_angle_, 3.14/2.0);
  nh_.param<double>("LaserMapper/min_angle", min_angle_, -3.14/2.0);
  nh_.param<double>("LaserMapper/minrange", min_range_, 0.001);
  nh_.param<double>("LaserMapper/maxrange", max_range_, 4);
  nh_.param<int>("LaserMapper/samplerate", samplerate_, 1);
  nh_.param<int>("LaserMapper/scan_processing_subsample", scan_subsample_, 0);
  nh_.param<int>("LaserMapper/mechanical_offset", mech_offset_, 0);

  nh_.param<int>("LaserMapper/lane_detection_left_msg/offset_height", offset_height_left_, -200);
  nh_.param<int>("LaserMapper/lane_detection_left_msg/offset_width", offset_width_left_, 0);
  nh_.param<int>("LaserMapper/lane_detection_right_msg/offset_height", offset_height_right_, -200);
  nh_.param<int>("LaserMapper/lane_detection_right_msg/offset_width", offset_width_right_, 500);
}

/**
 * @name PublishMap
 * @brief Joins the entire map together
 *        & Publishes the full_map
 * @return NONE
 */
void LaserMapper::PublishMap()
{
  nav_msgs::OccupancyGrid full_map;

  //Checks for lidar msg
  int n = floor(abs(min_angle_-laser_msg_.angle_min)/laser_msg_.angle_increment)+mech_offset_;
  double increment = (samplerate_)*laser_msg_.angle_increment;

  if (prev_header_.seq != laser_msg_.header.seq)
  {
    for (double i = min_angle_; i < max_angle_; i+= increment)
    {
      // Check for NaN ranges
      if (std::isnan (laser_msg_.ranges[n]) == false)
      {
        RayTracing(i, laser_msg_.ranges[n], inflate_obstacle_);
      }
      n += samplerate_;
    }
    prev_header_ = laser_msg_.header;
  }

  full_map.header.frame_id = "base_link";
  full_map.info.resolution = map_res_;
  full_map.info.width = map_width_;
  full_map.info.height = map_height_;
  full_map.info.origin.position.x = map_width_ / 2 * map_res_;
  full_map.info.origin.position.y = map_height_ * map_res_;
  full_map.info.origin.orientation =
             tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
  full_map.data.resize(map_width_*map_height_);

  for (int i = 0; i < map_width_*map_height_; i++)
  {
    full_map.data[i] = belief_map_[i];
  }

  //Checks for left lane msg
  {
    JoinOccupancyGrid(full_map, lane_detection_left_msg_, 
                      offset_height_left_, offset_width_left_);
  }

  //Checks for right lane msg
  {
    JoinOccupancyGrid(full_map, lane_detection_right_msg_,
                      offset_height_right_, offset_width_right_);
  } 

  map_pub_.publish(full_map);
  belief_map_.clear();
  belief_map_.resize(map_width_*map_height_);
}

/**
 * @name LidarCallback
 * @brief Obtains message sent by Lidar
 *        & Processes it to belief_map_
 * @param[in] msg: Lidar Message
 * @return NONE
 */
void LaserMapper::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  /* 
    Lidar, Left Lane, Right Lane callbacks all require
    DeleteValues - Deletes values of the 
                  belief map based on how much it moved
    StitchMap - Attaches the new map onto the front of the map
  */

  laser_msg_ = *msg;
  PublishMap();
}

/**
 * @name DetectLeftLaneCallback
 * @brief Retrieves the leftlanecallback message and stores it
 * @param[in] msg: Left lane message
 * @return NONE
 */
void LaserMapper::DetectLeftLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  lane_detection_left_msg_ = *msg;
}

/**
 * @name DetectRightLaneCallback
 * @brief Retrieves the rightlanecallback message and stores it
 * @param[in] msg: Right lane message
 * @return NONE
 */
void LaserMapper::DetectRightLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  lane_detection_right_msg_ = *msg;
}

/**
 * @name UpdateLaserMap
 * @brief Updates the belief map with 
 *        new occupany grid value if valid
 * @param[in] x: width comparator
 * @param[in] y: height comparator
 * @param[in] value: occupancy grid value
 * @return NONE
 */
void LaserMapper::UpdateLaserMap(const int& x, const int& y, const double& value) {
  if (abs(x) < map_width_/2 && y < map_height_ && y > 0) {
    int map_index = (map_width_/2 - x) + (map_height_ - y)*map_width_;
    belief_map_[map_index] = value;
  }
}

/**
 * @name CheckMap
 * @brief Checks value of the cell of occupancy grid 
 * @param[in] x: width comparator
 * @param[in] y: height comparator
 * @return double: value of occupany grid at belief map
 */
//TODO Change this function to have less redundant code (Duplicate with UpdateLaserMap)
double LaserMapper::CheckMap(const int& x, const int& y)
{
  if (abs(x) < map_width_/2 && y < map_height_ && y > 0)
  {
    int map_index = (map_width_/2 - x) + (map_height_ - y)*map_width_;
    return belief_map_[map_index];
  }
  return OBS_;
}

/**
 * @name RayTracing
 * @brief Checks value of the cell of occupancy grid 
 * @param[in] angle: width comparator
 * @param[in] range: height comparator
 * @param[in] inflate_factor: magnification value
 *  @return NONE
 */
void LaserMapper::RayTracing(const float& angle, const float& range, const int& inflate_factor)
{
  if (range < min_range_ || range > max_range_)
    return;
  
  int x1 = LASER_ORIENTATION_*round(sin(angle)*range/map_res_);
  int y1 = round(cos(angle)*range/map_res_);

  UpdateLaserMap(x1, y1, OBS_);

  int dx = abs(x1);
  int dy = abs(y1);
  int sx, sy;

  if (x1 > 0)
    sx = 1;
  else
    sx = -1;

  if (y1 > 0)
    sy = 1;
  else
    sy = -1;

  int err = dx-dy;
  int x = 0;
  int y = 0;

  while (true) {
    if (CheckMap(x, y) < OBS_)
      UpdateLaserMap(x, y, NO_OBS_);

    if (x == x1 && y == y1) {
      for (int i = -inflate_factor; i < inflate_factor; i++) {
        for (int j = -inflate_factor; j < inflate_factor; j++) {
          UpdateLaserMap(x+i, y+j, OBS_);
        }
      }
      return;
    }

    int e2 = 2*err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }

    if (x == x1 && y == y1) {
      for (int i = -inflate_factor; i < inflate_factor; i++)
        for (int j = -inflate_factor; j < inflate_factor; j++)
          UpdateLaserMap(x+i, y+j, OBS_);
        return;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  } 
}