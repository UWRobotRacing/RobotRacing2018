// Copyright [2016] University of Waterloo Robotics Team
// Robot Racing Path Planner Node
// Author: Ajay Kumar Singh
// Previous old code Author: Sirui Song, Jungwook Lee
// Date: 2016, 9th June

#ifndef PATHPLANNER2K16_H
#define PATHPLANNER2K16_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <limits> // for infinity

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

class PathPlanner2k16 {
public:
    PathPlanner2k16();
private:
    //functions
    void init();
    void getParams();
    //functions for path variables calculations
    std::vector< std::vector <double> > getAnglesAndWeights(double max_angle, int num_paths);
    void generateIdealPaths();  //paths without obstacle
    void generateRealPaths(); //paths with obstacle in real time
    int xyToMapIndex(double x, double y);
    std::vector<geometry_msgs::Point> rayTrace(double x0, double y0, double x1, double y1);
    void processMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void EnableCallBack(const std_msgs::Int8::ConstPtr& msg);
    bool isCellOccupied(int index);
    int checkLength(int angle_index);
    std_msgs::Float32 velocity(double dist, double steer);

    //ROS nodes, pub, sub, msgs & variables
    ros::NodeHandle node;
    ros::Subscriber map_sub;
    ros::Publisher vel_pub;
    ros::Publisher steer_pub;
    nav_msgs::OccupancyGrid::ConstPtr map_;  //map in 2d grid
    std_msgs::Float32 velMsg;   //velocity msg
    std_msgs::Float32 steerMsg; // steer msg
    ros::Subscriber enable_sub;
    std_msgs::Int8 enable;

    //map parameters
    int map_W_;
    int map_H_;
    double resolution_;

    //Car parameters
    int X_START_;
    int Y_START_;
    double CAR_WIDTH_;

    // Trajectory Rollout Path variables
    std::vector< std::vector <double> > anglesAndWeights;   //It is Nx2 matrix; 1st column is angle, 2nd column is weight
    std::vector< std::vector <double> > path_distance;
    std::vector< std::vector < std::vector <int> > > trajectory;
    int NUM_PATHS_;
    double PLANNER_VELOCITY_;
    int TRAJECTORY_STEPS_;
    int OBS_THRESHOLD_;
    double ANGLE_LIMIT_;
    double MIN_DISTANCE_;
    double dt_;
    double HORIZONTAL_LINE_RATIO_;
    int HORIZONTAL_LINE_THICKENESS_;

    // Desired Path Setup
    double ANGLE_WEIGHT_;
    double DISTANCE_WEIGHT_;
    double ANGLE_WEIGHT_DIFF_;
    double MAX_STEERING_ANGLE_;

    // Desired Velocity Setup
    double UX_DESIRED_;
    double ANGLE_VEL_DAMP_;
    double MIN_ACCELERATION_ANGLE_;
    double STRAIGHT_SPEED_;


    // Drag mode
    bool DRAG_MODE_;
    int DRAG_DURATION_;
    int DRAG_DURATION_NANO_;
    ros::Duration drag_duration_;
    bool start_recorded_; //used in drag mode to stop after a particular time
    ros::Time start_time_;

    bool check_traffic_;

    // Offset box params, obs in the box is ignored
    double min_offset_dist_;
    double MIN_STOPPING_DIST_;
    double DIST_COST_FACTOR_;

    //Debug variables
    bool VISUALIZATION_;
    bool DEBUG_ON_;
    void drawPath(ros::Publisher& pub, visualization_msgs::Marker& points,int id, int index, int R, int G, int B, float scale, float alpha);
    int all_path_marker_id;  //used as an id for drawing path using visualization_msgs
    int selected_path_marker_id;
    ros::Publisher all_path_pub;
    ros::Publisher selected_path_pub;
    std::vector <visualization_msgs::Marker> trajectory_marker_vector;  //used for visualizing selected_path
    visualization_msgs::Marker trajectory_points;
    //drawing axis
    /*visualization_msgs::Marker X_axis_marker_points;
    visualization_msgs::Marker Y_axis_marker_points;
    ros::Publisher X_axis_pub;
    ros::Publisher Y_axis_pub;
    int X_axis_marker_id;
    int Y_axis_marker_id;*/
    //drawing rayTrace
    std::vector <visualization_msgs::Marker> trajectory_marker_rayTrace;
    ros::Publisher rayTrace_pub;
    int trajectory_marker_rayTrace_id;
};

#endif
