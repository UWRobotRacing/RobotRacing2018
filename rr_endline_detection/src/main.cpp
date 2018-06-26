/** @file main.cpp
 * @brief Robot Racer endline detection main method
 *
 * This function subscribes to camera feed and then its only task is to call supervisor when endline is detected.
 *  
 *
 * @author Angela Gu (angegu)
 * @author Toni Ogunmade (oluwatoni)
 */

#include "endline_detection.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "endline_detection");
  ROS_INFO("Initializing endline_detection");	
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  EndlineCounter ec(nh);
  image_transport::Subscriber sub =
    it.subscribe(rr_sensor_topics::front_cam, 1, &EndlineCounter::ImgCb, &ec);
  ros::spin();
  return 0;
}
