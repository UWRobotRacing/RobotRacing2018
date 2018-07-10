/** @file laser_mapper.hpp
 *  @author Jungwook Lee
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

// ROS headers
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

#include <occupancy_grid_utils.hpp>

/** @brief joins two occupancy grids with the same resolution
 * 
 *  Joints 2 occupancy grid given a set of grid. The from_grid will be resized based on 
 *  the size of the old grid and the values will be added to the occupancy grid. In our
 *  case we use binary, so no need to actually add but do a bitwise or. Once the grid has 
 *  been joined, the pointer to the occupancy grid is returned.
 *  
 *  @param to_grid a grid to be joined
 *  @param from_grid destination grid to be joined
 *  @param offsetHeight an offset factored into the join
 *  @param offsetWidth an offset factored into the join
 *  @param start_val the starting value to slot into
 *         (Used for storing previous values)
 *  @return NONE
*/

void JoinOccupancyGrid(nav_msgs::OccupancyGrid &to_grid,
                        nav_msgs::OccupancyGrid &from_grid,
                        int offsetHeight, int offsetWidth)
{
  // Load The two grids
  if (&to_grid == NULL || &from_grid == NULL)
  {
    ROS_ERROR("JoinOccupancyGrid: Grid object NULL.");
    return;
  }

  int resolution = to_grid.info.resolution;

  // TODO(jungwook): Handle Resolution Change
  if (to_grid.info.resolution != from_grid.info.resolution)
  {
    ROS_ERROR("JoinOccupancyGrid: Resolution mismatch.");
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

/** @brief Returns index of given (i,j) from a grid given max_width and max_height
 * 
 *  Parameters: starting location of the grid and the height and width of the new grid.
 *  Starts from (0,0)
 *  --------> i
 *  ********* |
 *  ********* |
 *  ********* |
 *  ********* v j 
 *
 *  @parma i starting location of the grid(x-axis)
 *  @parma j starting location of the grid(x-axis)
 *  @parma max_width the width of the new grid.
*/
int ijToIndex(int i, int j, int max_width)
{
  // Debugging Code
  return floor((((max_width)*(j))-1)+i);
}
