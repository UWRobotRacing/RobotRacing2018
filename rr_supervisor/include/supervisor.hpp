/** @file supervisor.hpp
 *  @brief Supervisor prototypes
 *  @author Waleed Ahmed(w29ahmed)
 *  @competition IARRC 2018
 */

 // Include guard
#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


class Supervisor
{
  public:
    Supervisor();
    // Callback methods
    bool StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool CountLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void TrackSpeed(const geometry_msgs::TwistConstPtr& msg);
    void MonitorBattery(const std_msgs::Int8::ConstPtr& msg);
    void FinishRace();

    void IdleRobot();
    bool raceStarted;
  private:
    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Publisher null_lock_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber battery_sub_;
    ros::ServiceServer start_race_service_;
    ros::ServiceServer count_lap_service_;

    // Variables
    std_msgs::Bool bool_msg_;
    geometry_msgs::Twist twist_msg_;
    geometry_msgs::Vector3 null_vector_;
    std::string race_type_;
    clock_t begin_time_;
    int lap_count_;
    int twist_msg_count_;
    float speed_sum_;
    float race_time_;
    float average_speed_;
};

#endif  // SUPERVISOR_H

