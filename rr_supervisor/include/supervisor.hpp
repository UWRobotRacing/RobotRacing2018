/** @file supervisor.hpp
 *  @author Waleed Ahmed (w29ahmed)
 *  @competition IARRC 2018
 */

 // Include guard
#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

class Supervisor
{
  public:
    Supervisor();
    // Service callback methods
    bool startRace(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool countLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  private:
    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher twist_pub;
    ros::Publisher null_lock;
    ros::Publisher remove_null_lock;
    ros::ServiceServer start_race_service;
    ros::ServiceServer count_lap_service;

    // Variables
    std_msgs::Bool bool_msg;
    std::string race_type;
    int lap_count;
};

#endif  // SUPERVISOR_H

