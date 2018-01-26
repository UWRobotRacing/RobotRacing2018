// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Enabling Library
// Author: Ning Zhao and Shalin Upadhyay
// Date: 2015 06 11

/* A class that enable the publisher node send off true values to all its subscribers.
** The class should contain some member function to do the task.
** The variable is accessed by all the subscribers.
*/

#ifndef RR_LIBRARIES_ENABLESIGNALLER_H
#define RR_LIBRARIES_ENABLESIGNALLER_H

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

class EnableSignaller
{
    private:
        // private variables to set up the transports
        ros::Publisher pub;
        ros::NodeHandle nh;
        std::string topic_name_;
        std_msgs::Bool content;
    public:
        // later used for toggle the functionality
        // bool switch_state_;
        explicit EnableSignaller(std::string tn);

        void send_bool(bool data);
};

#endif  // RR_LIBRARIES_ENABLESIGNALLER_H
