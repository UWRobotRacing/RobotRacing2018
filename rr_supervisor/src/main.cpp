// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Supervisor Main
// Author: Waleed Ahmed
// Date: 2018 05 16

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor");
	ROS_INFO("Initializing supervisor node");
    ros::NodeHandle nh_;

    // The services below will be called from rr_traffic_light and rr_endline
    // TODO: Implement callbacks in a class method
    //ros::ServiceServer service = nh_.advertiseService("start_race", start);
    //ros::ServiceServer service = nh_.advertiseService("count_lap", lap_count);

	ros::spin();
    return 0;
}
