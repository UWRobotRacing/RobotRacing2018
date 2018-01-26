// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Laser Mapper Class
// Author: Sirui Song, Jungwook Lee, Raymond Kuo
// Date: 2015 06 04

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
    void processMap();
  private:
    // Methods
    void callBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    void visionCallBackLeft(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void visionCallBackRight(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void getParam();
    void initMap();
    void deleteMap();
    void updateLaserMap(int x, int y, double value);
    double checkMap(int x, int y);
    void rayTracing(float angle, float range, int inflate_factor);

    // ROS Variables
    ros::NodeHandle n_;
    ros::Publisher map_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber vis_left_sub_;
    ros::Subscriber vis_right_sub_;

    // Map Variables
    std::vector<double> belief_map_;
    nav_msgs::OccupancyGrid occu_msg_;
    nav_msgs::OccupancyGrid vis_left_msg_;
    nav_msgs::OccupancyGrid vis_right_msg_;
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

    // Vision Map Values
    int offset_height_left_;
    int offset_height_right_;
    int offset_width_left_;
    int offset_width_right_;

    // Debug Purposes
    bool DEBUG_;
    bool ready2Map_;
    bool ready2MapVisLeft_;
    bool ready2MapVisRight_;
    bool check_traffic_;

};

#endif  // LASERMAPPER_H

