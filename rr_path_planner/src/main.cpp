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

ros::Subscriber enable_sub;
bool enable;

/** @brief handles the enable signal
 *  @param enable_msg the enable signal
 *  @return NONE
 */
void enable_cb(const std_msgs::Bool& enable_msg)
{
    enable = enable_msg.data;
}

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

  ros::Rate loop_rate(25);  //** frequency should be get using GetParams for Synchronization
  enable = false;

  // Debug Code
  // enable = true;

  PathPlanner TrajectoryGenerator;

  ros::spin();

  ROS_INFO("Trajectories Generation Completed");
  return 0;
}

