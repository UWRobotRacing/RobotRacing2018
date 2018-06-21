/**
 * @brief simulates the robots odometry with some gaussian noise tossed in for fun
 * 
 * @file simulated_odom_node.cpp
 * @author Toni Ogunmade(oluwatoni)
 * @date 2018-05-04
 * @competition IARRC 2018
 */
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "simulated_odom.hpp"

/**
 * @brief starts the simulated odometry node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rr_simulated_odometry");
  SimulatedOdom simulated_odometry;

  ros::spin();
  return 0;
}