// Copyright [2015] University of Waterloo Robotics Team
/***********************************************************************************
  Robot Racing PID Controller
  Date:12/06/15

  This node takes in a velocity command and a heading from the path planner and
  uses a PID controller to ensure that the heading and velocity driven do not differ.
  Information is taken in from encoders and an IMU.

  Author: Sirui Song, Archie Lee, Jungwook Lee

************************************************************************************/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#define dt 0.02

class Controller
{
  private:
    std::string name_;

    bool PID_DEBUG_;

    double Kp_;
    double Kd_;
    double Ki_;
    double Kf_;

    double val_reference_;
    double val_measured_;
    double val_error_;
    double de_val_error_;
    double in_val_error_;
    int cmd_;

    std::string pub_topic_;
    std::string reference_topic_;
    std::string measured_topic_;

    bool enable_;
    int prevMode_;
    int mode_;
    std_msgs::Int32 val_cmd_;
    // std_msgs::Float32 steer_cmd;

    ros::NodeHandle nh_;

    ros::Timer controller_loop_;

    ros::Publisher val_pub_;

    ros::Subscriber enable_sub_;
    ros::Subscriber measured_sub_;
    ros::Subscriber reference_sub_;
    ros::Subscriber state_sub_;

    void getParam(ros::NodeHandle nh);

    void enableCb(const std_msgs::Bool::ConstPtr& enable_msg);
    void callback(const std_msgs::Float32::ConstPtr& measured_msg);
    void referenceCb(const std_msgs::Float32::ConstPtr& reference_msg);
    void stateCb(const std_msgs::Int8::ConstPtr& state_msg);
    // void steer_cb(/* Need some parameters */);

  public:
    Controller();
    Controller(std::string name);

    // double timeElapsed();
    void PID(const ros::TimerEvent&);
    // float lateral_error_calculation(double velocity, double dt);
};

#endif  // CONTROLLER_H
