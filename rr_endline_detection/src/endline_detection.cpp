
#include "endline_detection.hpp"

EndlineCounter::EndlineCounter (ros::NodeHandle nh) : it(nh) 
{
	pub = nh.advertise<std_msgs::Bool>("endline",1);
	this->state = BEGINNING;
}

// void EndlineCounter::img_callback(const sensor_msgs::ImagePtr& msg) {
	
// 	// process image
	
	
// 	// publish 
// 	// if(this->state & PAST_END) {
// 	// 	this->pub.publish(true);
// 	// } else {
// 	// 	this->pub.publish(false);
// 	// }
// }

void EndlineCounter::find_el_state(){}