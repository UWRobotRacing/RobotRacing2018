/** @file laser_mapper.hpp
 *  @author Sirui Song
 *  @author Jungwook Lee
 *  @author Raymond Kuo
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#ifndef LASERMAPPER_H
#define LASERMAPPER_H

#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

/*
 *  Callback Class for laser to Occumpancy Grid Format/OpenCV Format
 */

class LaserMapper
{
  public:
    LaserMapper();
    void ProcessMap();
  private:
    // Methods
    void CallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    void DetectLeftLane(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void DetectRightLane(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void GetParam();
    void InitMap();
    void DeleteMap();
    void UpdateLaserMap(int x, int y, double value);
    double CheckMap(int x, int y);
    void RayTracing(float angle, float range, int inflate_factor);

    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber lane_detection_left_sub_;
    ros::Subscriber lane_detection_right_sub_;

    // Map Variables
    std::vector<double> belief_map_;
    nav_msgs::OccupancyGrid occu_msg_;
    nav_msgs::OccupancyGrid lane_detection_left_msg_;
    nav_msgs::OccupancyGrid lane_detection_right_msg_;
    sensor_msgs::LaserScan laser_msg_;

    std::string occupancy_grid_name_;
    std::string laser_scan_name_;
    std_msgs::Header prev_header_;

    // Map Parameters
    double map_res_;
    double map_orientation_;
    int map_W_;
    int map_H_;

    // Scan Parameters
    double max_angle_;
    double min_angle_;
    double minrange_;
    double maxrange_;
    int samplerate_;
    int inflate_obstacle_;
    int scan_subsample_;
    int mech_offset_;
    double LASER_ORIENTATION_;

    // Map Values
    double NO_OBS_;
    double OBS_;
    double UNKNOWN_;
    double OBS_SCALE_;

    // lane_detection Map Values
    int offset_height_left_;
    int offset_height_right_;
    int offset_width_left_;
    int offset_width_right_;

    // Debug Purposes
    bool DEBUG_;
    bool ready2Map_;
    bool ready2Maplane_detectionLeft_;
    bool ready2Maplane_detectionRight_;

};

#endif  // LASERMAPPER_H

