// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Supervisor Main
// Author: Waleed Ahmed
// Date: 2018 05 16

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor");
	ROS_INFO("Initializing supervisor node");
    ros::NodeHandle nh_;

    // Publishers for the twist multiplexer
    ros::Publisher null_pub = nh_.advertise<geometry_msgs::Twist>("null_mux", 1);
    ros::Publisher null_lock = nh_.advertise<std_msgs::Bool>("no_movement", 1);

    // Message objects
    std_msgs::Bool bool_msg;
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Vector3 null_vector;

    // Stuff messages with data
    bool_msg.data = true;
    null_vector.x = 0.0; null_vector.y = 0.0; null_vector.z = 0.0;
    twist_msg.linear = null_vector;
    twist_msg.linear = null_vector;

    // Publish messages
    null_pub.publish(bool_msg);
    null_lock.publish(twist_msg);

    // The services below will be called from rr_traffic_light and rr_endline
    // TODO: Implement callbacks in a class method
    //ros::ServiceServer service = nh_.advertiseService("start_race", start);
    //ros::ServiceServer service = nh_.advertiseService("count_lap", lap_count);

	ros::spin();
    return 0;
}
