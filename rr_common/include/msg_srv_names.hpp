/** @file comms.hpp
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 *
 *  @brief defines the topic and service names across the project
 */

#ifndef __LANE_DETECTION_PROCESSOR_HPP
#define __LANE_DETECTION_PROCESSOR_HPP

#include <string.h>

static std::string rr = "/rr_vehicle/";
static std::string raw = "_cam/image_raw";

namespace rr_sensor_topics{
  static std::string left_cam = rr + "left" + raw;
  static std::string front_cam = rr + "front" + raw;
  static std::string right_cam = rr + "right" + raw;
  static std::string imu = rr + "imu";
  static std::string laser = rr + "laserscan";
  static std::string joint_states = rr + "joint_states";
  static std::string simulated_odom = rr + "simulated_odom";
}

namespace rr_cmd_topics{
  static std::string vel_cmd = rr + "vel_cmd";
  static std::string joy_cmd = rr + "joy_cmd";
  static std::string path_planner_cmd = rr + "pp_cmd";
}

namespace rr_signal_srvs{
  static std::string traffic_signal;
  static std::string endline_signal;
}

namespace rr_processed_topics{
  static std::string fused_odom;
  static std::string fused_map = rr + "map";
  static std::string planned_path;
}

#endif //__LANE_DETECTION_PROCESSOR_HPP