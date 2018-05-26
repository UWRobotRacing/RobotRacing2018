// Copyright [2018] University of Waterloo Robotics Team
// Robot Racing Supervisor Main
// Author: Waleed Ahmed
// Date: 2018 05 16

#include <supervisor.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor");
	ROS_INFO("Initializing supervisor node");
    ros::NodeHandle nh_;

    // Instantiate Supervisor object
    Supervisor supervisor;

	ros::spin();
    return 0;
}
