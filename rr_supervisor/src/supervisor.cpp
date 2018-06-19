// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Supervisor Node class implementation
// Author: Waleed Ahmed
// Date: 2018 05 25

#include <supervisor.hpp>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <sstream>

// Supervisor Class constructor
Supervisor::Supervisor()
{
    // Initialize variables
    lap_count = 0;
    speed_sum = 0.0;
    twist_msg_count = 0;
    nh_.param<std::string>("Launch/race_type", race_type, "drag");

    // Setup publishers for twist multiplexer
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/Supervisor/twist_mux", 1);
    null_lock = nh_.advertise<std_msgs::Bool>("/Supervisor/no_movement", 1);
    remove_null_lock = nh_.advertise<std_msgs::Bool>("/Supervisor/enable_movement", 1);

    // Subscribe to vehicle's velocity publisher to calculate average speed
    cmd_sub = nh_.subscribe("/rr_vehicle/vel_cmd", 1, &Supervisor::trackSpeed, this);

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
bool Supervisor::startRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // Publish a bool that the twist multiplexer will pick up that will allow
    // allow path planner and joystick messages to be published
    bool_msg.data = true;
    remove_null_lock.publish(bool_msg);

    // Start timer in order to log race time
    begin_time = clock();

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

    if (res.success) {
        finishRace();
    }
    return true;
}

/**
 * @brief subscriber callback method for tracking vehicle speed
 * @param msg twist message published to vehicle
 */
void Supervisor::trackSpeed(const geometry_msgs::TwistConstPtr& msg)
{
    twist_msg_count += 1;
    speed_sum += sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
}

/**
 * @brief handle end of race events, namely logging race metrics
 */
void Supervisor::finishRace()
{
    // Calculate average speed and elapsed time since start of race
    average_speed = speed_sum / twist_msg_count;
    race_time = float(clock() - begin_time) / CLOCKS_PER_SEC;

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
    file_name << "~/" << race_type << "_" << date_string << ".txt";
    race_metrics_file.open(file_name.str().c_str());

    // Write to file
    std::ostringstream metrics;

    metrics << "Date: " << date_string << "\n";
    metrics << "Race type: " << race_type << "\n";
    metrics << "Race time: " << race_time << "\n";
    metrics << "Average speed: " << average_speed << "\n";

    race_metrics_file << metrics.str();
    race_metrics_file.close();

    ros::shutdown();
}
