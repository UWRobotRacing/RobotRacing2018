/***************************************************************************
  Robot Racer Laser Mapping
  Sirui Song, Jungwook Lee, Raymond Kuo
  Date:11/07/14
  Copyright 2015 UWRT/Wavelab
 
  This function takes in the scan messages, and translate it into occupancy grid. If vision occupancy grid is available, and the option is turned on, this mapper will only publish messages when new vision data is available.

  ** If enable is off, then nothing is published! Resets the laser map.
  ** Always assumes initial vehicle position is at (0,0) = mid bottom of the image
  ** Grid size changes with Vision Occupancy Grid
  ** Parameters are loaded in the robotracing trajectory launch node. 
  ** The node keeps two maps, vision and laser. If vision node is not available
  Vision node is not updated with obstacles (empty 0s)

****************************************************************************/
// To be implemented:
/*
 *  TODO:1. Make sure the parameters for min_angle/max_angle work
 *       2. Check if any of the functions can be optimized
 *       3. Conversion to OpenCV matrix as an option/ or image_raw
 *       4. Make the code easier to read
 */

#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "LaserMapper.h"

#define DEBUG false


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rr_mapper");
  LaserMapper laserProcessor;

  // Subscribe to Enable
  bool enable = false;
  ros::Subscriber enable_sub;
  ros::NodeHandle nh;

  ROS_INFO("Mapper: Laser Node Initialized");

  // Debug Code
#ifdef DEBUG
  enable = true;
#endif
  ros::Rate spin_rate(25);
  while (ros::ok())
  {
    laserProcessor.processMap();
    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
