// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Supervisor Node class implementation
// Author: Waleed Ahmed
// Date: 2018 05 25

#include <ros/ros.h>
#include <supervisor.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Trigger.h>

// Supervisor Class constructor
Supervisor::Supervisor()
{
    // Initialize variables
    lap_count = 0;
    nh_.param<std::string>("Launch/race_type", race_type, "drag");

    // Setup publishers for twist multiplexer
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/Supervisor/twist_mux", 1);
    null_lock = nh_.advertise<std_msgs::Bool>("/Supervisor/no_movement", 1);
    remove_null_lock = nh_.advertise<std_msgs::Bool>("/Supervisor/enable_movement", 1);

    // Setup service servers
    start_race_service = nh_.advertiseService("/Supervisor/start_race", &Supervisor::startRace, this);
    count_lap_service  = nh_.advertiseService("/Supervisor/count_lap", &Supervisor::countLap, this);

    // Message objects
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Vector3 null_vector;

    // Stuff messages with data
    bool_msg.data = true;
    null_vector.x = 0.0; null_vector.y = 0.0; null_vector.z = 0.0;
    twist_msg.linear = null_vector;
    twist_msg.angular = null_vector;

    // Publish messages
    twist_pub.publish(twist_msg);
    null_lock.publish(bool_msg);
}

/**
 * @brief service callback method for starting the race
 * @param res response that will be sent back to client
 */
bool Supervisor::startRace(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;

    // Publish a bool that the twist multiplexer will pick up that will allow
    // allow path planner and joystick messages to be published
    bool_msg.data = true;
    remove_null_lock.publish(bool_msg);

    res.message = "Race started";
    return true;
}

/**
 * @brief service callback method for tracking lap count
 * @param res response that will be sent back to client
 */
bool Supervisor::countLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (race_type == "drag") {
        lap_count += 1;
        res.success = true;
    }
    else if (race_type == "circuit") {
        if (lap_count < 3) {
            lap_count += 1;
            if (lap_count == 3) {
                res.success = true;
            }
            else{
                res.success = false;
            }
        }
    }
    return true;
}