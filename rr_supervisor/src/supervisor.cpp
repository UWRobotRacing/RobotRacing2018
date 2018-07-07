/** @file supervisor.cpp
 *  @brief Supervisor class implementation
 *
 *  Services provided:
 *    /Supervisor/start_race
 *    /Supervisor/count_lap
 *
 *  Topics Published:
 *    /Supervisor/cmd
 *    /Supervisor/no_movement
 *    /Supervisor/enable_movement
 *
 *  Topics Subscribed:
 *    /rr_vehicle/vel_cmd
 *    /arduino/battery_state
 *
 *  @author Waleed Ahmed(w29ahmed)
 *  @competition IARRC 2018
 */

#include <supervisor.hpp>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <sstream>

/**
 * @brief Supervisor class constructor
 */
Supervisor::Supervisor()
{
  // Initialize variables
  lap_count_ = 0;
  speed_sum_ = 0.0;
  twist_msg_count_ = 0;
  nh_.param<std::string>("Launch/race_type", race_type_, "drag");
  bool_msg_.data = true;
  null_vector_.x = 0.0; null_vector_.y = 0.0; null_vector_.z = 0.0;
  twist_msg_.linear = null_vector_;
  twist_msg_.angular = null_vector_;

  // Setup publishers for twist multiplexer
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/Supervisor/cmd", 1);
  null_lock_ = nh_.advertise<std_msgs::Bool>("/Supervisor/no_movement", 1);
  remove_null_lock_ = nh_.advertise<std_msgs::Bool>("/Supervisor/enable_movement", 1);

  // Setup subscribers to monitor battery and speed
  cmd_sub_ = nh_.subscribe("/rr_vehicle/vel_cmd", 1, &Supervisor::TrackSpeed, this);
  battery_sub_ = nh_.subscribe("/arduino/battery_state", 1, &Supervisor::MonitorBattery, this);

  // Setup service servers
  start_race_service_ = nh_.advertiseService("/Supervisor/start_race", &Supervisor::StartRace, this);
  count_lap_service_  = nh_.advertiseService("/Supervisor/count_lap", &Supervisor::CountLap, this);

  // Publish messages
  null_lock_.publish(bool_msg_);
  twist_pub_.publish(twist_msg_);
}

/**
 * @brief service callback method for starting the race
 * @param res response that will be sent back to client
 * @return bool
 */
bool Supervisor::StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // Publish a bool that the twist multiplexer will pick up that will allow
  // allow path planner and joystick messages to be published
  bool_msg_.data = true;
  remove_null_lock_.publish(bool_msg_);

  // Start timer in order to log race time
  begin_time_ = clock();

  ROS_INFO("Race started");

  return true;
}

/**
 * @brief service callback method for tracking lap count
 * @param res response that will be sent back to client
 * @return bool
 */
bool Supervisor::CountLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (race_type_ == "drag")
  {
    lap_count_ += 1;
    res.success = true;
    ROS_INFO("Drag race complete! shutting down");
  }
  else if (race_type_ == "circuit")
  {
    if (lap_count_ < 3)
    {
      lap_count_ += 1;
      if (lap_count_ == 3)
      {
        res.success = true;
        ROS_INFO("Circuit race complete! shutting down");
      }
      else
      {
        res.success = false;
        ROS_INFO("Lap %d of %d complete!", lap_count_, 3);
      }
    }
  }

  if (res.success)
  {
    this->FinishRace();
  }
  return true;
}

/**
 * @brief subscriber callback method for tracking vehicle speed
 * @param msg twist message published to vehicle
 * @return void
 */
void Supervisor::TrackSpeed(const geometry_msgs::TwistConstPtr& msg)
{
  twist_msg_count_ += 1;
  speed_sum_ += sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
}

/**
 * @brief subscriber callback method for monitoring battery
 * @param msg battery percentage as an int published from arduino
 * @return void
 */
void Supervisor::MonitorBattery(const std_msgs::Int8::ConstPtr& msg)
{
  if (msg->data <= 10)
  {
    ROS_INFO("Battery is less than 10 percent, ending race...");
    this->FinishRace();
  }
}

/**
 * @brief handle end of race events, namely logging race metrics and stopping the robot
 * @return void
 */
void Supervisor::FinishRace()
{
  // Calculate average speed and elapsed time since start of race
  average_speed_ = speed_sum_ / twist_msg_count_;
  race_time_ = float(clock() - begin_time_) / CLOCKS_PER_SEC;

  // Construct a timestamp with the following format:
  // Year_Month_day--Hours_minutes_seconds
  time_t rawtime;
  struct tm * time_info;
  char date_string[50];
  time(&rawtime);
  time_info = localtime(&rawtime);
  strftime(date_string, 50, "%Y_%m_%d--%H_%M_%S", time_info);

  // Open a file to write to
  std::ofstream race_metrics_file;
  std::ostringstream file_name;
  file_name << "~/" << race_type_ << "_" << date_string << ".txt";
  race_metrics_file.open(file_name.str().c_str());

  // Write to file
  std::ostringstream metrics;

  metrics << "Date: " << date_string << "\n";
  metrics << "Race type: " << race_type_ << "\n";
  metrics << "Race time: " << race_time_ << "\n";
  metrics << "Average speed: " << average_speed_ << "\n";

  race_metrics_file << metrics.str();
  race_metrics_file.close();

  // Publish null vector to twist multiplexer and lock out any other messages
  // from being published, which will stop the robot
  twist_pub_.publish(twist_msg_);
  null_lock_.publish(bool_msg_);
}
