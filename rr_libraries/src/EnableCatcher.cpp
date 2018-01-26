// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Enabling Library
// Author: Ning Zhao and Shalin Upadhyay
// Date: 2015 06 11

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <rr_libraries/EnableCatcher.h>

EnableCatcher::EnableCatcher(std::string tn)
{
    if (!ros::ok())
    {
        ROS_INFO("Failed to initiate the msg_sender");
    }
    else
    {
        topic_name_ = tn;

        sub = nh.subscribe(topic_name_, 1, &EnableCatcher::CatchCallback, this);

        pub = nh.advertise<std_msgs::Bool>("Enable Signal Advertise", 1);
    }
}

void EnableCatcher::CatchCallback(const std_msgs::Int8::ConstPtr &msg)
{
    BoolBuffer.data = msg->data;

    std::cout << "Heard from " << topic_name_ << "[" << msg->data << "]";
    pub.publish(BoolBuffer);
}
