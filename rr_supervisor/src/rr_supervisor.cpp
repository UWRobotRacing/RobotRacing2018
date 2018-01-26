// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Supervisor Node
// Author: Juichung Kuo and Shalin Upadhyay
// Date: 2014 07 09

/*TODO:
- Detect state of arduino and other nodes
*/

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>

#include <EnableCatcher.h>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <rr_supervisor.h>

Supervisor::Supervisor()
{
    control_state_ = 0;
    prev_state_ = false;
    enable_state_.data = false;
    control_state_name_ = "vehState";
    tl_sub_name_ = "/drag_race_cv/traffic_light_go";
    enable_pub_name_ = "/RR1/Enable";

    control_state_sub_ = nh_.subscribe(control_state_name_, 1, &Supervisor::controlStateCallback, this);
    tl_sub_ = nh_.subscribe(tl_sub_name_, 1, &Supervisor::trafficLightCallback, this);

    enable_pub_ = nh_.advertise<std_msgs::Bool>(enable_pub_name_, 1);
}

void Supervisor::controlStateCallback(const std_msgs::Int8::ConstPtr &msg)
{
    control_state_ = msg->data;

    if (prev_state_ != enable_state_.data && enable_state_.data  == true)
    {
        prev_state_ = enable_state_.data;
    }

    switch (control_state_)
    {
        case 0:  // Estop
            enable_state_.data = false;
            break;
        case 1:  // RC
            enable_state_.data = false;
            break;
        case 2:  // Auto
            enable_state_.data = prev_state_;
            break;
        default:
            enable_state_.data = false;
    }
    enable_pub_.publish(enable_state_);
}

void Supervisor::trafficLightCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (control_state_ == 2 && msg->data == true)
    {
        enable_state_.data = msg->data;
        enable_pub_.publish(enable_state_);
    }
}
