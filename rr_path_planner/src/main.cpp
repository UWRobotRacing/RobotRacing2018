// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Path Planner Node
// Author: Jungwook Lee
// Date: 2015 07 09

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

#include <PathPlanner2k16.h>

ros::Subscriber enable_sub;
bool enable;

void enable_cb(const std_msgs::Bool& enable_msg)
{
    enable = enable_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generation");
  ros::NodeHandle n;
  ROS_INFO("trajectories Generation Initalized");

  ros::Rate loop_rate(25);  //** frequency should be get using getParams for Synchronization
  enable = false;

  // Debug Code
  // enable = true;

  PathPlanner2k16 TrajGenerator;

  while (ros::ok())
  {
    //PathPlanner TrajGenerator;
    ros::spinOnce();
  }

  ROS_INFO("Trajectories Generation Completed");
  return 0;
}

