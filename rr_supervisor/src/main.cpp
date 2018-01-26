// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Supervisor Main
// Author: Juichung Kuo and Shalin Upadhyay
// Date: 2015 06 04

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <rr_supervisor.h>
#include <EnableCatcher.h>

int main(int argc, char **argv)
{
  // INITIALIZATION
  ros::init(argc, argv, "rr_supervisor");

  Supervisor sup;
  EnableCatcher ec("/enable");

  ros::Rate loop_rate(20);

  // ROS main Loop
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
