// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Supervisor Class
// Author: Juichung Kuo and Shalin Upadhyay 
// Date: 2015 06 04

// Supervisor node controls the feedback for the current state of
// the car (ESTOP, RC, AUTONOMOUS). Also triggers the required nodes
// based on the state of the traffic light.

#ifndef RR_SUPERVISOR_H
#define RR_SUPERVISOR_H

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

class Supervisor
{
    private:
        // control state: 0 = Estop; 1 = RC state; 2 = Auto
        int control_state_;

        // Stores the previous state of the vehicle
        bool prev_state_;

        // Topic name for control_state_sub_
        std::string control_state_name_;
        // Topic name for tl_sub_
        std::string tl_sub_name_;
        // Topic name for enable_pub_
        std::string enable_pub_name_;

        std_msgs::Bool enable_state_;

    	ros::NodeHandle nh_;

    	// Subscriber for control state from Arduino
        ros::Subscriber control_state_sub_;
        // Subscriber for traffic light sensing node
        ros::Subscriber tl_sub_;

        // Publisher for enalbe signal
        ros::Publisher enable_pub_;

        void controlStateCallback(const std_msgs::Int8::ConstPtr &msg);
        void trafficLightCallback(const std_msgs::Bool::ConstPtr &msg);

    public:
        Supervisor();
};
#endif  // RR_SUPERVISOR_H
