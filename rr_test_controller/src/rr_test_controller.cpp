// Copyright [2015] University of Waterloo Robotics Team
/***********************************************************************************
  Robot Racing PID Controller Tester
  Date:12/06/15

  This program simulates the path planner by publishing a velocity and heading using
  that topic name and allows a person to test the PID controller.

  Author: Archie Lee

************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "time.h"
#include <unistd.h>

ros::Publisher vel_pub;
ros::Publisher steer_pub;
ros::Subscriber state_sub;

std_msgs::Float32 vel;
std_msgs::Float32 steer;

int prevMode_ = 0, mode_ = 0;

// void velChanger(std_msgs::Float32 &vel)
// {
//   time_t begin, timer;
//   time(&begin);
//   time(&timer);
//   while (difftime(timer, begin) < 15)
//   {
//     if (difftime(timer, begin) < 2)
//       vel.data = 0.0;
//     else if (difftime(timer, begin) < 10)
//       vel.data = 2.0;
//     else
//       vel.data = 1.15;
//     time(&timer);
//     vel_pub.publish(vel);
//     usleep(100000);
//   }
// }

// void stateCb(const std_msgs::Int8::ConstPtr& state_msg)
// {
//   mode_ = state_msg->data;
//   if (prevMode_ != 2 && mode_ == 2)
//   {
//     velChanger(vel);
//   }
//   prevMode_ = mode_;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_test");
  ros::NodeHandle n;
  ROS_INFO("Starting test controller node");
  int count = 0;

  vel_pub = n.advertise<std_msgs::Float32>("/PathPlanner/vel_cmd", 1);
  steer_pub = n.advertise<std_msgs::Float32>("/PathPlanner/steer_cmd", 1);
  //state_sub = n.subscribe("/arduino/vehicle_state", 1, &stateCb); 

  ros::Rate loop_rate(20);

  steer.data = 0.0;
  vel.data = 1.25; // should not be over 10 m/s

  sleep(10);

  while (ros::ok() && count < 20)
  {
    vel_pub.publish(vel);
    steer_pub.publish(steer);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}

