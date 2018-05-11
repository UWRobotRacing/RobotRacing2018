/**
 * @brief header file for the  simulated odometry node
 * 
 * @file simulated_odom.hpp
 * @author Toni Ogunmade(oluwatoni)
 * @date 2018-05-04
 */

#ifndef SIMULATED_ODOM_H_
#define SIMULATED_ODOM_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class SimulatedOdom{
public:
  SimulatedOdom();
private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber joint_sub_;
  ros::Time then_;
  tf::TransformBroadcaster br_;
  enum joints_{
    LEFT_STEERING = 2,
    RIGHT_STEERING = 5,
    FRONT_LEFT_WHEEL = 0,
    FRONT_RIGHT_WHEEL = 3,
    REAR_LEFT_WHEEL = 1,
    REAR_RIGHT_WHEEL = 4 
  };
  double angular_vel_sd_;
  double linear_vel_x_sd_;
  double linear_vel_y_sd_;
  double wheel_diameter_;
  double wheel_to_wheel_dist_;
  nav_msgs::Odometry odom_;
  void jointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
};
#endif