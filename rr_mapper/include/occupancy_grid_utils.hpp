/** @file occupancy_grid_utils.hpp
 *  @author Sirui Song
 *  @author Jungwook Lee
 *  @author Raymond Kuo
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#ifndef RR_LIBRARIES_OCCUPANCYGRIDUTILS_H
#define RR_LIBRARIES_OCCUPANCYGRIDUTILS_H

// ROS headers
#include <nav_msgs/OccupancyGrid.h>

void JoinOccupancyGrid(nav_msgs::OccupancyGrid &to_grid,
                       nav_msgs::OccupancyGrid &from_grid,
                       int offsetHeight, int offsetWidth);
int ijToIndex(int i, int j, int max_width);

#endif  // RR_LIBRARIES_OCCUPANCYGRIDUTILS_H