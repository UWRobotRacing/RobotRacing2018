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

#include <rr_libraries/EnableSignaller.h>

EnableSignaller::EnableSignaller(std::string tn)
{
    if (!ros::ok())
    {
        ROS_INFO("Failed to initiate the EnableSignaller");
    }
    else
    {
        topic_name_ = tn;

        pub = nh.advertise<std_msgs::Bool>(topic_name_, 1);
    }
}

void EnableSignaller::send_bool(bool data)
{
    content.data = data;

    pub.publish(content);
    std::cout << "Sent to " << topic_name_ << "[" << content.data << "]";
}
