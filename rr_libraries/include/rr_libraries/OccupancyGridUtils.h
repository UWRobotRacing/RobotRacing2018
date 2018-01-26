// Copyright [2015] University of Waterloo Robotics Team
// Occupancy Grid Utility Functions
// Author: Sirui Song, Jungwook Lee, Raymond Kuo
// Date: 2015 06 04

#ifndef RR_LIBRARIES_OCCUPANCYGRIDUTILS_H
#define RR_LIBRARIES_OCCUPANCYGRIDUTILS_H

// ROS headers
#include <nav_msgs/OccupancyGrid.h>

// TO IMPLEMENT
void joinOccupancyGrid(nav_msgs::OccupancyGrid &to_grid,
                        nav_msgs::OccupancyGrid &from_grid,
                        int offsetHeight, int offsetWidth);

// nav_msgs::OccupancyGrid resizeOccupancyGrid(nav_msgs::OccupancyGrid grid);
int ijToIndex(int i, int j, int max_width);

#endif  // RR_LIBRARIES_OCCUPANCYGRIDUTILS_H

