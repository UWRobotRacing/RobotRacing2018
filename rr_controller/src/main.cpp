// Copyright [2015] University of Waterloo Robotics Team
/***********************************************************************************
  Robot Racing PID Controller
  Date:12/06/15

  This program calls a constructor for the controller class and loops to run the
  controller.

  Author: Sirui Song, Archie Lee, Jungwook Lee

************************************************************************************/

#include <controller.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  Controller vel_controller("velocity");
  // Controller steer_controller;
  ROS_INFO("Starting PID controllers");

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}

