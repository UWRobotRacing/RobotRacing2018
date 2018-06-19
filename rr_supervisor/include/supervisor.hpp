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
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


class Supervisor
{
  public:
    Supervisor();
    // Callback methods
    bool startRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool countLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void trackSpeed(const geometry_msgs::TwistConstPtr& msg);
    void finishRace();
  private:
    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher twist_pub;
    ros::Publisher null_lock;
    ros::Publisher remove_null_lock;
    ros::Subscriber cmd_sub;
    ros::ServiceServer start_race_service;
    ros::ServiceServer count_lap_service;

    // Variables
    std_msgs::Bool bool_msg;
    std::string race_type;
    int lap_count;
    int twist_msg_count;
    float speed_sum;
    clock_t begin_time;
    float race_time;
    float average_speed;
};

#endif  // SUPERVISOR_H

