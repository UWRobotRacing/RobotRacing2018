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
#include <tf/transform_listener.h>

/*
 *  Callback Class for laser to Occumpancy Grid Format/OpenCV Format
 */

class LaserMapper
{
  public:
    LaserMapper();
    ~LaserMapper();

    // Publishers
    void ProcessMap();

  private:    
    // Subscribers
    void LidarCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    void DetectLeftLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void DetectRightLaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // Methods
    void GetParam();
    void InitMap();
    void DeleteMap();
    void UpdateLaserMap(const int& x, const int& y, const double& value);
    double CheckMap(int x, int y);
    void RayTracing(float angle, float range, int inflate_factor);
    void StitchMap(std::string ref_name);

    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber lane_detection_left_sub_;
    ros::Subscriber lane_detection_right_sub_;
    // tf::TransformListener position_listener_;

    // Map Variables
    std::vector<double> belief_map_;
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
    // double NO_OBS_;
    // double OBS_;
    // double UNKNOWN_;
    const double OBS_SCALE_ = 1;

    // lane_detection Map Values
    int offset_height_left_;
    int offset_height_right_;
    int offset_width_left_;
    int offset_width_right_;

    // Debug Purposes
    bool DEBUG_;
    bool ready2Map_ = true;
    bool ready2Maplane_detectionLeft_ = false;
    bool ready2Maplane_detectionRight_ = false;

    enum CellState {
        NO_OBS_ = 0,
        OBS_ = 100,
        UNKNOWN_ = -1
    };

};

#endif  // LASERMAPPER_H

