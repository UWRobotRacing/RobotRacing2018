/** @file main.cpp
 *  @author Jungwook Lee
 *  @author Sirui Song
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <vector>

#include <path_planner.h>

/** @brief starts the path_planner node
 *  This node doesn't act until it gets the enable signal
 *
 *  @return NONE
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generation");
  ros::NodeHandle n;
  ROS_INFO("trajectories Generation Initalized");
  PathPlanner TrajectoryGenerator;
  //TODO(oluwatoni) remove the delay
  //ros::Duration(6).sleep();
  ros::spin();

  ROS_INFO("Trajectories Generation Completed");
  return 0;
}
