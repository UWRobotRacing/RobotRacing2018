/** @file main.cpp
 *  @brief Supervisor main method, initialize supervisor node
 *  @author Waleed Ahmed(w29ahmed)
 *  @competition IARRC 2018
 */

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
