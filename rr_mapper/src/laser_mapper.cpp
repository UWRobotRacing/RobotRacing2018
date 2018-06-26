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
  lane_detection_left_sub_ = nh_.subscribe("/output_point_list_left", 1, &LaserMapper::DetectLeftLaneCallback, this);
  lane_detection_right_sub_ = nh_.subscribe("/output_point_list_right", 1, &LaserMapper::DetectRightLaneCallback, this);

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
  belief_map_.resize(map_W_*map_H_, UNKNOWN_);
  ROS_INFO("Map Initialized with UNKNOWN, Height: %d, Width: %d", map_H_, map_W_);
}

/**
 * @name GetParam
 * @brief Obtains ROS Params
 * @return NONE
 */
void LaserMapper::GetParam() {
  nh_.param<std::string>("LaserMapper/Occupancy_Grid_Name", occupancy_grid_name_, "/map");
  nh_.param<std::string>("LaserMapper/Laser_Scan_Name", laser_scan_name_, "/scan");

  nh_.param<double>("LaserMapper/map_res", map_res_, 0.01);
  nh_.param<int>("LaserMapper/map_W", map_W_, 800);
  nh_.param<int>("LaserMapper/map_H", map_H_, 300);
  nh_.param<double>("LaserMapper/map_orientation", map_orientation_, M_PI);
  nh_.param<double>("LaserMapper/LASER_ORIENTATION", LASER_ORIENTATION_, -1);

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
 *        & Publishes the full_map_
 * @return NONE
 */
void LaserMapper::PublishMap() {
  nav_msgs::OccupancyGrid full_map_;

  full_map_.header.frame_id = "/map";
  full_map_.info.resolution = map_res_;
  full_map_.info.width = map_W_;
  full_map_.info.height = map_H_;
  full_map_.info.origin.position.x = map_W_/2*map_res_;//-map_W_/2*map_res_; //map_W_/2*map_res_
  full_map_.info.origin.position.y = 0;//-map_H_/2*map_res_; //map_H_*map_res_
  full_map_.info.origin.orientation =
             tf::createQuaternionMsgFromRollPitchYaw(M_PI, -1*M_PI, 0);
  full_map_.data.resize(map_W_*map_H_);

  //Deletes Values based on new position
  ShiftMap(belief_map_);

  //Checks for lidar msg
  if(lidar_msg_call_){
    int n = floor(abs(min_angle_-laser_msg_.angle_min)/laser_msg_.angle_increment)+mech_offset_;    
    double increment = (samplerate_)*laser_msg_.angle_increment;

    if (prev_header_.seq != laser_msg_.header.seq) {
      for (double i = min_angle_; i < max_angle_; i+= increment) {
        // Check for NaN ranges
        if (std::isnan (laser_msg_.ranges[n]) == false) {
          RayTracing(i, laser_msg_.ranges[n], 0);
        }
        n += samplerate_;
      }
      prev_header_ = laser_msg_.header;
    }

    for (int i = 0; i < map_W_*map_H_; i++) {
      double weight = std::min(100.0, OBS_SCALE_*belief_map_[i]);
      weight = std::max(0.0, weight);

      full_map_.data[i] =
      std::max(static_cast<int>(weight), static_cast<int>(belief_map_[i]));
    }
    lidar_msg_call_ = false;
  }

  //Checks for left lane msg
  if (left_msg_call_) {
    JoinOccupancyGrid(full_map_, lane_detection_left_msg_, 
                      offset_height_left_, offset_width_left_);
    left_msg_call_ = false;
  }
  else 
    ROS_WARN("No Left Name Data Detected");

  //Checks for right lane msg
  if (right_msg_call_) {
    JoinOccupancyGrid(full_map_, lane_detection_right_msg_,
                      offset_height_right_, offset_width_right_);
    right_msg_call_ = false;
  } 
  else
    ROS_WARN("No Right Lane Data Detected");
  
  map_pub_.publish(full_map_);
}

/**
 * @name LidarCallback
 * @brief Obtains message sent by Lidar
 *        & Processes it to belief_map_
 * @param[in] msg: Lidar Message
 * @return NONE
 */
void LaserMapper::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /* 
    Lidar, Left Lane, Right Lane callbacks all require
    DeleteValues - Deletes values of the 
                  belief map based on how much it moved
    StitchMap - Attaches the new map onto the front of the map
  */

  laser_msg_ = *msg;
  lidar_msg_call_ = true;
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
  ROS_INFO("Left Data Received!");
  left_msg_call_ = true;
}

/**
 * @name DetectRightLaneCallback
 * @brief Retrieves the rightlanecallback message and stores it
 * @param[in] msg: Right lane message
 * @return NONE
 */
void LaserMapper::DetectRightLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  lane_detection_right_msg_ = *msg;
  ROS_INFO("Right Data Receieved!");
  right_msg_call_ = true;
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
  ROS_INFO("Center Data Received!");
  if (abs(x) < map_W_/2 && y < map_H_ && y > 0) {
    int map_index = (map_W_/2 - x) + (map_H_ - y)*map_W_;
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
double LaserMapper::CheckMap(const int& x, const int& y) {
  if (abs(x) < map_W_/2 && y < map_H_ && y > 0) {
    int map_index = (map_W_/2 - x) + (map_H_ - y)*map_W_;
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
void LaserMapper::RayTracing(const float& angle, const float& range, const int& inflate_factor) {
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

/**
 * @name ShiftMap
 * @brief Shifts the map based on x y movement
 * @param[in] prev_map: map that needs to be updated
 * @return Updates the prev_map with shifting map
 */
void LaserMapper::ShiftMap(std::vector<int> prev_map) {
  tf::StampedTransform transform;
  try {
    position_listener_.lookupTransform("/odom", "/base_link",
                              ros::Time(0), transform);

  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return;
  }

  // Assumes previous value is given
  if (prev_x_ != 0 && prev_y_ != 0) {
    /*Gains the difference in x and y transition
      If the diff value is negative it is 
      either fill left or bottom (vice versa)
    */
    int diff_x = transform.getOrigin().x() - prev_x_;
    int diff_y = transform.getOrigin().y() - prev_y_;

    //Filling UNKNOWN for x
    if (diff_x > 0) {
      for(int i = 0; i < map_H_; i++) {
        std::vector<int> temp(map_W_);
        std::copy(prev_map.begin(), prev_map.begin()+map_W_, temp.begin());
        std::rotate(temp.begin(), temp.begin()+abs(diff_x), temp.end());
        std::fill(temp.rbegin(), temp.rbegin()+abs(diff_x), UNKNOWN_);
        std::copy(temp.begin(), temp.end(), prev_map.begin());
      }

      // Replaces functionality with std::fill()
      // for(int i = 1; i <= map_H_; i++) {
      //   for(int j = 1; j <= abs(diff_x); j++) {
      //     prev_map[(j*i)-1] = UNKNOWN_;
      //   }
      // }

    }
    else {
      for(int i = 0; i < map_H_; i++) {
        std::vector<int> temp(map_W_);
        std::copy(prev_map.begin(), prev_map.begin()+map_W_, temp.begin());
        std::rotate(temp.begin(), temp.begin()+abs(diff_x), temp.end());
        std::fill(temp.begin(), temp.begin()+abs(diff_x), UNKNOWN_);
        std::copy(temp.begin(), temp.end(), prev_map.begin());
      }

      // Replaces functionality with std::fill()
      // for(int i = map_H_; i >= 1; i--) {
      //   for(int j = abs(diff_x); j >= 1; j--) {
      //     prev_map[(j*i)-1] = UNKNOWN_;
      //   }
      // }
    }
    
    //Filling UNKNOWN for y
    if (diff_y > 0) {
      //'Rotates' the map dragging all values 
      //By abs(diff_y)*map_W_ amount
      std::rotate(prev_map.begin(), 
          prev_map.begin()+(abs(diff_y)*map_W_), 
          prev_map.end());

      for(int i = 0; i < abs(diff_y)*map_W_; i++) {
        prev_map[i] = UNKNOWN_;
      }
    }
    else {
      std::rotate(prev_map.rbegin(), 
          prev_map.rbegin()+(abs(diff_y)*map_W_), 
          prev_map.rend());

      for(int i = (abs(diff_y)*map_W_)-1; i >= 0; i--) {
        prev_map[i] = UNKNOWN_;
      }
    }
  }

  prev_x_ = transform.getOrigin().x();
  prev_y_ = transform.getOrigin().y();

  //Rotation
  double diff_ang = transform.getRotation().getAngle() - prev_ang_;
  prev_map = RotateMap(prev_map, diff_ang);
  prev_ang_ = transform.getRotation().getAngle();
}

/**
 * @name RotateMap
 * @brief Rotates the map based on angular rotation
 * @param[in] curr_map: current map to analyze
 * @param[in] new_ang: angular difference from previous location
 * @return rot_map: newly rotated map
 */
std::vector<int> LaserMapper::RotateMap(std::vector<int> curr_map, double new_ang){
  std::vector<LaserMapper::CellEntity> cell_map;
  std::vector<int> rot_map;
  //Return a empty Cell Map if no value is input
  if(curr_map.empty()){
    ROS_ERROR("LaserMapper::RotateMap: Map is empty!");
    return rot_map;
  }

  //Generate CellEntity Vector & Rotate
  for(int i = 0; i < map_H_; i++){
    for(int j = 1; j <= map_W_; j++){
      int curr_x = j - map_W_/2;
      int curr_y = map_H_ - i;
      LaserMapper::CellEntity curr_en;
      curr_en.val = curr_map.at(i*map_H_ + j - 1);
      curr_en.length = sqrt(curr_x*curr_x +
                        curr_y*curr_y);
      curr_en.angle = atan2(curr_y, curr_y) + new_ang;
      curr_en.xloc = rint(curr_en.length*cos(curr_en.angle));
      curr_en.yloc = rint(curr_en.length*sin(curr_en.angle));
      cell_map.push_back(curr_en);
    }
  }

  //Generates a map of unknwons same size as the belief map
  rot_map.resize(map_W_*map_H_, UNKNOWN_);

  //Fills in values based on criteria  
  for(int i = 0; i < rot_map.size(); i++){
    LaserMapper::CellEntity curr_en = cell_map.at(i);
    //Checks for bounds of x & y
    if((curr_en.xloc >= 0 && curr_en.xloc <= map_W_) && 
        (curr_en.yloc >= 0 && curr_en.yloc <= map_H_)){
      rot_map[i] = curr_en.val;
    }
  }

  return rot_map;
}