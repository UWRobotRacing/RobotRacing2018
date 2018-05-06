/** @file laser_mapper.cpp
 *  @author Jungwook Lee
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

// CPP
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

// Local
#include "laser_mapper.hpp"
#include <occupancy_grid_utils.hpp>

/**
 * @brief initiliazes the LaserMapper class
 * @return NONE
 */
LaserMapper::LaserMapper()
{
  // Load Parameters
  GetParam();

  // PUBLISHERS
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_name_, 1);
  
  // SUBSCRIBERS
  scan_sub_ = nh_.subscribe(laser_scan_name_, 1, &LaserMapper::LidarCallBack, this);
  lane_detection_left_sub_ = nh_.subscribe("/output_point_list_left", 1, &LaserMapper::DetectLeftLaneCallback, this);
  lane_detection_right_sub_ = nh_.subscribe("/output_point_list_right", 1, &LaserMapper::DetectRightLaneCallback, this);

  // Initialize an occupancy grid
  InitMap();
}

/**
 * @brief destructs the LaserMapper class
 * @return NONE
 */
LaserMapper::~LaserMapper() {
  DeleteMap();
}

/** @brief get the rosparams
 *  @return NONE
 */
void LaserMapper::GetParam()
{
  nh_.param<std::string>("LaserMapper/Occupancy_Grid_Name", occupancy_grid_name_, "/map");
  nh_.param<std::string>("LaserMapper/Laser_Scan_Name", laser_scan_name_, "/scan");

  // nh_.param<double>("LaserMapper/NO_OBS", NO_OBS_, 0);
  // nh_.param<double>("LaserMapper/OBS", OBS_, 200);
  // nh_.param<double>("LaserMapper/UNKNOWN", UNKNOWN_, -1);
  // nh_.param<double>("LaserMapper/OBS_SCALE", OBS_SCALE_, 1);
  // nh_.param<int>("LaserMapper/INFLATE_OBS", inflate_obstacle_, 2);

  nh_.param<double>("LaserMapper/map_res", map_res_, 0.01);
  nh_.param<int>("LaserMapper/map_W", map_W_, 800);
  nh_.param<int>("LaserMapper/map_H", map_H_, 300);
  nh_.param<double>("LaserMapper/map_orientation", map_orientation_, M_PI);
  nh_.param<double>("LaserMapper/LASER_ORIENTATION", LASER_ORIENTATION_, -1);

  nh_.param<double>("LaserMapper/max_angle", max_angle_, 3.14/2.0);
  nh_.param<double>("LaserMapper/min_angle", min_angle_, -3.14/2.0);
  nh_.param<double>("LaserMapper/minrange", minrange_, 0.001);
  nh_.param<double>("LaserMapper/maxrange", maxrange_, 4);
  nh_.param<int>("LaserMapper/samplerate", samplerate_, 1);
  nh_.param<bool>("LaserMapper/DEBUG", DEBUG_, false);
  nh_.param<int>("LaserMapper/scan_processing_subsample", scan_subsample_, 0);
  nh_.param<int>("LaserMapper/mechanical_offset", mech_offset_, 0);

  nh_.param<int>("LaserMapper/lane_detection_left_msg/offset_height", offset_height_left_, -200);
  nh_.param<int>("LaserMapper/lane_detection_left_msg/offset_width", offset_width_left_, 0);
  nh_.param<int>("LaserMapper/lane_detection_right_msg/offset_height", offset_height_right_, -200);
  nh_.param<int>("LaserMapper/lane_detection_right_msg/offset_width", offset_width_right_, 500);
}

/** @brief callback for the left lane grid
 *  @param msg the occupancy grid message
 *  @return NONE
 */
void LaserMapper::DetectLeftLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  lane_detection_left_msg_ = *msg;

  if (!ready2Maplane_detectionLeft_)
    ready2Maplane_detectionLeft_ = true;
  // Assumes laser is much faster than lane_detection data
  // Note: modify this function after camera integration
}

/** @brief callback for the right lane grid
 *  @param msg the occupancy grid message
 *  @return NONE
 */
void LaserMapper::DetectRightLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  lane_detection_right_msg_ = *msg;

  if (!ready2Maplane_detectionRight_)
    ready2Maplane_detectionRight_ = true;
  // Assumes laser is much faster than lane_detection data
  // Note: modify this function after camera integration
}

/** @brief initiliazes an occupancy grid and sets all the cells to unknown
 *  @return NONE
 */
void LaserMapper::InitMap()
{
  belief_map_.resize(map_W_*map_H_, UNKNOWN_);
  ROS_INFO("Map Initialization Done. Size(%d, %d).", map_H_,map_W_);
}

/** @brief empties the underlying vector of the map
 *  @return NONE
 */
void LaserMapper::DeleteMap()
{
  belief_map_.clear();
}

/** @brief sets cell at location x and y to value passed in
 *  @param x the x-axis value
 *  @param y the y-axis value
 *  @param value the new value for the cell on the map
 *  @return NONE
 */ 
void LaserMapper::UpdateLaserMap(const int& x, const int& y, const double& value)
{
  // In occupancy grid scale, not world scale
  // Change it to update using vectors, and then put it into a 1D array.
  if (abs(x) < map_W_/2 && y < map_H_ && y > 0)
  {
    int Map_index = (map_W_/2-x)+(map_H_ - y)*map_W_;
    belief_map_[Map_index] = value;
    // ROS_INFO("map update with value %f", value);
  }
}

/** @brief returns the value of the cell with the cell coordinates
 *  @param x the x-axis value
 *  @param y the y-axis value
 *  @return NONE
 */
double LaserMapper::CheckMap(int x, int y)
{
  if (abs(x) < map_W_/2 && y < map_H_ && y > 0)
  {
    int Map_index = (map_W_/2-x)+(map_H_ - y)*map_W_;
    return belief_map_[Map_index];
  }
  return OBS_;
}

/** @brief places a single line of the laser data unto the map
 *  @param angle the line 
 *  @param range the distance reading of the laser
 *  @param inflate_factor the number of cells used to inflate the map
 *  @return NONE
 */
void LaserMapper::RayTracing(float angle, float range, int inflate_factor)
{
  if (range < minrange_)
    return;
  else if (range > maxrange_)
    return;

  // x1, y1 is are the end position of the ray
  // If dis(x0,y0, x1,y1) < range, it hit an obstacle, else, its unknown

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

  while (true)
  {
    if (CheckMap(x, y) < OBS_)
      UpdateLaserMap(x, y, NO_OBS_);

    if (x == x1 && y == y1)
    {
      for (int i = -inflate_factor; i < inflate_factor; i++) {
        for (int j = -inflate_factor; j < inflate_factor; j++) {
          UpdateLaserMap(x+i, y+j, OBS_);
        }
      }
      return;
    }

    int e2 = 2*err;
    if (e2 > -dy)
    {
      err = err -dy;
      x = x + sx;
    }

    if (x == x1 && y == y1)
    {
      for (int i = -inflate_factor; i < inflate_factor; i++)
        for (int j = -inflate_factor; j < inflate_factor; j++)
          UpdateLaserMap(x+i, y+j, OBS_);
        return;
    }
    if (e2 < dx)
    {
      err = err + dx;
      y = y+sy;
    }
  }
}

/** @brief copies the recieved lidar data
 *  @param msg the laser scan data
 *  @return NONE
 */
void LaserMapper::LidarCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  static int scan_subsample_counter = 0;
  ++scan_subsample_counter;
  if (msg->angle_min > min_angle_)
  {
    min_angle_= msg->angle_min;
  }
  else if (msg->angle_max < max_angle_)
  {
    max_angle_ = msg->angle_max;
  }
  if (scan_subsample_counter >= scan_subsample_)
  {
    scan_subsample_counter = 0;
  }

  if (scan_subsample_counter != 0)
  {
    return;
  }

  laser_msg_ = *msg;  

  // ROS_INFO ("Mapper: Copied laser data");

  if (!ready2Map_)
    ready2Map_ = true;

  // Assumes laser is much faster than lane_detection data
  // Note: modify this function after camera integration
}

/** @brief combines the sensor data together into a single map and 
 *  then publishes it
 * 
 *  if laser data is saved it starts the process of creating a map
 *  it raytraces the lidar data over the occupancy grid and adds the lanes 
 *  detected if they're available
 *
 *  @return NONE
 */
void LaserMapper::ProcessMap()
{
  if (!ready2Map_)
    return;

  nav_msgs::OccupancyGrid grid_msg_;

  int n = floor(abs(min_angle_-laser_msg_.angle_min)/laser_msg_.angle_increment)+mech_offset_;    
  // for (double i = laser_msg.angle_min; i < laser_msg.angle_max; i+= increment)
  double increment = (samplerate_)*laser_msg_.angle_increment;
  // ROS_INFO ("Value of pre seq is : %d", prev_header_.seq);
  // ROS_INFO ("Value of cur seq is : %d", laser_msg_.header.seq);
  if (prev_header_.seq != laser_msg_.header.seq)
  {
    // ROS_INFO ("Value of obs is : %f", OBS);
    // ROS_INFO ("Value of no_obs is : %f", NO_OBS);

    for (double i = min_angle_; i < max_angle_; i+= increment)
    {
      // Check for NaN ranges
      if (std::isnan (laser_msg_.ranges[n]) == false)
      {
        RayTracing(i, laser_msg_.ranges[n], 0);
        // ROS_INFO ("1");
      }
      n+= samplerate_;
    }
    prev_header_ = laser_msg_.header;
  }

  grid_msg_.header.frame_id = "/map";
  grid_msg_.info.resolution = map_res_;
  grid_msg_.info.width = map_W_;
  grid_msg_.info.height = map_H_;
  grid_msg_.info.origin.position.x = map_W_/2*map_res_;//-map_W_/2*map_res_; //map_W_/2*map_res_
  grid_msg_.info.origin.position.y = map_H_/2*map_res_;//-map_H_/2*map_res_; //map_H_*map_res_
  grid_msg_.info.origin.orientation =
             tf::createQuaternionMsgFromRollPitchYaw(M_PI, -1*M_PI, 0);
  grid_msg_.data.resize(map_W_*map_H_);

  for (int i = 0; i < map_W_*map_H_; i++)
  {
    double weight = std::min(100.0, OBS_SCALE_*belief_map_[i]);
    weight = std::max(0.0, weight);

    grid_msg_.data[i] =
    std::max(static_cast<int>(weight), static_cast<int>(belief_map_[i]));
  }

  // Join the lane_detection occupancy and the laser occupancy together.
  if (ready2Map_ && ready2Maplane_detectionLeft_ && ready2Maplane_detectionRight_)
  {
    // ROS_INFO("Joining maps");
    JoinOccupancyGrid(grid_msg_, lane_detection_left_msg_, offset_height_left_, offset_width_left_);
    JoinOccupancyGrid(grid_msg_, lane_detection_right_msg_, offset_height_right_, offset_width_right_);
  }

  map_pub_.publish(grid_msg_);
  DeleteMap();
  InitMap();
}

void LaserMapper::StitchMap(std::string ref_name) {
  ROS_INFO("Im a test!");

  //Listens for tf and uses lookup transform 
  /*
  tf::StampedTransform transform;
  try{
     listener_.lookupTransform("/turtle2", "/turtle1",  
                              ros::Time(0), transform);
   }
   catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
   }
   ...
   transform.getOrigin().y();
   transform.getOrigin().x();
  */
  //Should also do the distance calculation
}