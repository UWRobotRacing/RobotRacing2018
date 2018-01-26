// Copyright [2015] University of Waterloo Robotics Team
// Occupancy Grid Utility Functions
// Author: Jungwook Lee
// Date: 2015 06 04

// ROS headers
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

#include <rr_libraries/OccupancyGridUtils.h>
// TO IMPLEMENT
// Function Name

/*
Joints 2 occupancy grid given a set of grid. The from_grid will be resized based on 
the size of the old grid and the values will be added to the occupancy grid. In our
case we use binary, so no need to actually add but do a bitwise or. Once the grid has 
been joined, the pointer to the occupancy grid is returned.

Parameters: a set of grid to be joined, and offset in x and y directions.
*/

void joinOccupancyGrid(nav_msgs::OccupancyGrid &to_grid,
                        nav_msgs::OccupancyGrid &from_grid,
                        int offsetHeight, int offsetWidth)
{
  // Load The two grids
  if (&to_grid == NULL || &from_grid == NULL)
  {
    ROS_ERROR("joinOccupancyGrid: Grid object NULL.");
    return;
  }

  // if (offsetHeight < 0 || offsetWidth < 0 )
  // {
  //   ROS_ERROR("joinOccupancyGrid: Input offsets are negative.");
  //   return;
  // }

  int resolution = to_grid.info.resolution;

  // TODO(jungwook): Handle Resolution Change
  if (to_grid.info.resolution != from_grid.info.resolution)
  {
    ROS_ERROR("joinOccupancyGrid: Resolution mismatch.");
    return;
  }

  // Debug Code
  // ROS_INFO("Map Resolution: %f", to_grid.info.resolution);
  // ROS_INFO("Map width: %d", to_grid.info.width);
  // ROS_INFO("Map Height: %d", to_grid.info.height);

  // ROS_INFO("Map Resolution: %f", from_grid.info.resolution);
  // ROS_INFO("Map width: %d", from_grid.info.width);
  // ROS_INFO("Map Height: %d", from_grid.info.height);


  // Resize if necessary
  //int start_index = ijToIndex(offsetWidth, offsetHeight, from_grid.info.width);

  //ROS_INFO("Current Index: %d", start_index);

  // Copy content to the to the original grid
  for (int j = 0; j < from_grid.info.height; j++)
  {
    for (int i = 0; i < from_grid.info.width; i++)
    {
      //ROS_INFO("i+offset = %d", i+offsetWidth);
      //ROS_INFO("j+offset = %d", j+offsetHeight);
      // Check for valid size before modifications
      if (i+offsetWidth < to_grid.info.width && j+offsetHeight < to_grid.info.height)
      {
          // Do operation
            int index = ijToIndex(i+offsetWidth, j+offsetHeight, to_grid.info.width);
            int from_index = ijToIndex(i, j, from_grid.info.width);
            if (from_grid.data[from_index] > 0) // OBS are larger than 0
              to_grid.data[index] = from_grid.data[from_index];
            //to_grid.data[index] = 200;
          // Debug code
      }
    }
  }

  // Send back the pointer to the map
  return;
}

/*
Resizes the occupancy grid based on the given parameters.

Parameters: starting location of the grid and the height and width of the new grid.

nav_msgs::OccupancyGrid* resizeOccupancyGrid(nav_msgs::OccupancyGrid* grid, int height,
                                            int width, int startX, int startY)
{
}
*/

/*
Returns index of given (i,j) from a grid given max_width and max_height

Parameters: starting location of the grid and the height and width of the new grid.
Starts from (0,0)
--------> i
********* |
********* |
********* |
********* v j 
*/
int ijToIndex(int i, int j, int max_width)
{
  // Debugging Code
  // ROS_INFO("i,j,max_width: %d, %d, %d", i,j,max_width);
  // ROS_INFO("output : %f",floor((max_width-1)*j)+(i));
  return floor((((max_width)*(j))-1)+i);
}
