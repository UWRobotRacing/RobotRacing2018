
#include "endline_detection.hpp"

EndlineCounter::EndlineCounter (ros::NodeHandle nh) : it(nh) 
{
	pub = nh.advertise<std_msgs::Bool>("endline",1);
	state = BEGINNING;
}

void EndlineCounter::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("hello world!");
	//publish 
	// if(state & PAST_END) {
	// 	pub.publish(true);
	// } else {
	// 	pub.publish(false);
	// }
}

void EndlineCounter::find_el_state(){}