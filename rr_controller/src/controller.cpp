// Copyright [2015] University of Waterloo Robotics Team
/***********************************************************************************
  Robot Racing PID Controller
  Date:12/06/15

  This node takes in a velocity command and a heading from the path planner and
  uses a PID controller to ensure that the heading and velocity driven do not differ.
  Information is taken in from encoders and an IMU.
      * enable is constantly ON at the moment due to enableCb() not receiving a
        message from supervisor but will be changed

  Author: Sirui Song, Archie Lee, Jungwook Lee

************************************************************************************/

#include <controller.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <iostream>

void Controller::getParam(ros::NodeHandle nh)
{
  nh.param<bool>("PIDController/"+name_+"/PID_DEBUG", PID_DEBUG_, false);

  nh.param<double>("PIDController/"+name_+"/Kp_", Kp_, 0);
  nh.param<double>("PIDController/"+name_+"/Kd_", Kd_, 0);
  nh.param<double>("PIDController/"+name_+"/Ki_", Ki_, 0);
  nh.param<double>("PIDController/"+name_+"/Kf_", Kf_, 0);

  nh.param<std::string>("PIDController/"+name_+"/pub_topic", pub_topic_, "/controller/pub_topic");
  nh.param<std::string>("PIDController/"+name_+"/reference_topic", reference_topic_, "/controller/reference_topic");
  nh.param<std::string>("PIDController/"+name_+"/measured_topic", measured_topic_, "/controller/measured_topic");
}

Controller::Controller(std::string name)
{
  name_ = name;

  // Get parameters from yaml file
  getParam(nh_);

  val_error_ = 0;
  de_val_error_ = 0;
  in_val_error_ = 0;
  
  // prev_t_ = ros::Time::now();

  controller_loop_ = nh_.createTimer(ros::Duration(dt), &Controller::PID, this);

  // Publishers
  // steer_pub = nh.advertise<std_msgs::Float32>("/controller/steer_cmd");
  val_pub_ = nh_.advertise<std_msgs::Int32>(pub_topic_, 1);

  // Subscribers
  // steer_sub = nh.subscribe("/path_planner/steer_angle", 1, steer_cb);
  reference_sub_ = nh_.subscribe(reference_topic_, 1, &Controller::referenceCb, this);
  enable_sub_ = nh_.subscribe("/supervisor/enable", 1, &Controller::enableCb, this);
  measured_sub_ = nh_.subscribe(measured_topic_, 1, &Controller::callback, this);
  state_sub_ = nh_.subscribe("/arduino/vehicle_state", 1, &Controller::stateCb, this);
}

/*
double Controller::timeElapsed()
{
  ros::Time current = ros::Time::now();
  // Calculate time difference and convert to seconds
  dt_ = (current - prev_t_).toNSec()*1E-9;
  // Update previous time
  prev_t_ = current;

  return dt_;
}
*/

void Controller::PID(const ros::TimerEvent&)
{
  // To avoid integrator wind up, the enable pin should be tied to the Estop status
  // Calculate error
  double tmp_error = val_reference_ - val_measured_;
  de_val_error_ = (val_error_ - tmp_error)/dt;
  in_val_error_ = std::max(0.0, std::min(50.0, in_val_error_ + Ki_*tmp_error*dt));  // Limit to some % of maximum
  val_error_ = tmp_error;

  // Call PID --> turns into PWM
  // PWM to m/s --> y (m/s) = 0.07 * x (PWM) - 2.289 for any x >= 50
  cmd_ = std::max(0, std::min(175, static_cast<int>(Kp_*val_error_ + in_val_error_ + Kd_*de_val_error_ + Kf_*val_reference_)));

  // Publish command
  val_cmd_.data = cmd_;
  val_pub_.publish(val_cmd_);
}

/*float Controller::lateral_error_calculation (double velocity, double dt) {

  double yawdesired= atan(cam_lat_error/lookahead);
  double yawerror = cam_errorheading - yawdesired;
  float cmd_steering = 0;

  if (PID_DEBUG)
  {
    std::cout << "yawdesired = " << yawdesired << std::endl; 
    std::cout << "lateral error = " << cam_lat_error << std::endl;
    std::cout << "errorHeading = " << cam_errorheading << std::endl;
  }

  // Wrapping error
  if (yawerror > PI)
  {
    yawerror = 2*PI - yawerror;
  }
  else if (yawerror < -PI)
  {
    yawerror = 2*PI + yawerror;
  }  

  if (PID_DEBUG)
  {
    std::cout << "yawError after wrap = " << yawerror << std::endl;
  }

  // To avoid integrator wind up, listen to enable, when disabled, integrator is off.  
  // std::cout << "ENABLE  = " << enable << ", dt = " << dt <<std::endl;

  if (enable && dt > 0)
  {
    dheading_error = (yawerror-heading_error)/dt;
    heading_error = yawerror;
    if (velocity > 0)
      iheading_error = in_vel_error + yawerror;

    cmd_steering = -PID(heading_error, dheading_error, iheading_error, 0, Kp_lat, Kd_lat, Ki_lat, 0);
    // std::cout << "cmd_steering = " << cmd_steering << std::endl;
  }
  else
  {
    iheading_error = 0;
    cmd_steering = 0;
  }

  return cmd_steering;
}*/

// Takes in desired velocity --> map to pwm?
void Controller::referenceCb(const std_msgs::Float32::ConstPtr& reference_msg)
{
  val_reference_ = reference_msg->data;
}

/*
void Controller::steer_cb(Need some parameters)
{

}
*/

void Controller::stateCb(const std_msgs::Int8::ConstPtr& state_msg)
{
  prevMode_ = mode_;
  mode_ = state_msg->data;
  if (prevMode_ != 2 && mode_ == 2)
  {
    in_val_error_ = 0;
  }
}

void Controller::enableCb(const std_msgs::Bool::ConstPtr& enable_msg)
{
  // enable_ = enable_msg->data;
  enable_ = true;
}

void Controller::callback(const std_msgs::Float32::ConstPtr& measured_msg)
{
  val_measured_ = measured_msg->data;
}
