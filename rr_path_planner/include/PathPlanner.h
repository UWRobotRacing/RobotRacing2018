// Copyright [2015] University of Waterloo Robotics Team
// Robot Racing Path Planner Node
// Author: Jungwook Lee
// Date: 2015 07 09

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <vector>

class PathPlanner
{
  public:
    PathPlanner();
    void generatePaths();   //** real time path with obstacle
  private:
    void getParam();
    bool checkMap(double x, double y); //** haven't used this function.. instead used checkLength() function;
    std::vector<double> generateAngles(int numLines, double MAX_STEER);
    int checkLength(int angle_index);
    void callBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void initializePaths(); //** ideal paths
    int xyToMapIndex(double x, double y);
    bool checkBox(int Map_index);
    std::vector<geometry_msgs::Point> rayTrace(double x0, double y0, double x1, double y1);
    void pushPoints(visualization_msgs::Marker& marker, double x, double y);
    bool isOBS(int index);
    void drawChosenLine(ros::Publisher& pub, visualization_msgs::Marker& points, int index,int id, int color_R, int color_G, int color_B, float scale, float alpha);                                                                    //** draws selected path with max distance it can move;
    void drawLine(ros::Publisher& pub, visualization_msgs::Marker& points, 
            int id, int color_R, int color_G, int color_B, float scale, float alpha); //** draws all path and selected path depends on input;
    /* 
     *  Each vector have index of the paths.
     */
        std::vector<double> angles_;
        std::vector<double> angles_weight_;
        std::vector< std::vector < std::vector <int> > > trajectory_index_;
        std::vector< std::vector <double> >  distance_;
    std::vector<visualization_msgs::Marker> trajectory_points;
    visualization_msgs::Marker all_paths;
    visualization_msgs::Marker chosen_path;

    //**********************************************
    int map_W_;
    int map_H_;
    double resolution_;

    //********************************************************
    /* Trajectory Rollout parameters */
    int NUM_PATHS_;
    double PLANNER_VELOCITY_;
    int TRAJECTORY_STEPS_;
    int OBS_THRESHOLD_;
    double ANGLE_LIMIT_;
    double MIN_DISTANCE_;
        int X_START_;
        int Y_START_;
    double dt_;
    double HORIZONTAL_LINE_RATIO_;

    // Desired Path Setup
    double ANGLE_WEIGHT_;
    double DISTANCE_WEIGHT_;
    double ANGLE_WEIGHT_DIFF_;
        double CAR_WIDTH_;
    double MAX_STEERING_ANGLE_;

    // Desired Velocity Setup
    double UX_DESIRED_;
    double ANGLE_VEL_DAMP_;
    double MIN_ACCELERATION_ANGLE_;
    double STRAIGHT_SPEED_;
<<<<<<< HEAD
    //***************************************************************
=======

    // For Trajectory Rollout debugging
    bool VISUALIZATION_;
    bool DEBUG_ON_;
    int line_id_;
    int chosen_path_line_id_;
    bool traj_drawn_;

    // Offset box params, obs in the box is ignored
    double min_offset_dist_;
    double MIN_STOPPING_DIST_;
    double DIST_COST_FACTOR_;

    // End line Detection variables
    bool start_line_detected_;

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher steer_pub_;
    ros::Publisher all_path_pub_;
    ros::Publisher chosen_path_pub_;
    ros::Publisher start_line_pub_;
    std_msgs::Float32 cmdAngleMsg_;
    std_msgs::Float32 cmdVelMsg_;

    nav_msgs::OccupancyGrid::ConstPtr map_;
<<<<<<< HEAD

>>>>>>> f441e361b6fa7c6003fc3961b287c77f441401b0
    // Enable Catcher
    EnableCatcher enableCatcher_;

    // Drag mode
    bool DRAG_MODE_;
    int DRAG_DURATION_;
    int DRAG_DURATION_NANO_;
    ros::Duration drag_duration_;
    bool start_recorded_;
    ros::Time start_time_;
    ros::Time current_time_;

    bool check_traffic_;

<<<<<<< HEAD
    //***************************************************************************
    // End line Detection variables
    bool start_line_detected_;
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher cmd_pub_;    //** I'm using vel_pub
    ros::Publisher steer_pub_;
    std_msgs::Float32 cmdAngleMsg_;
    std_msgs::Float32 cmdVelMsg_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    // Offset box params, obs in the box is ignored
    double min_offset_dist_;
    double MIN_STOPPING_DIST_;
    double DIST_COST_FACTOR_;
    //******************************************************************************
    
    // Test variables
    std::vector<int> test_path_;
    std::vector<visualization_msgs::Marker> test_markers_;
    ros::Publisher test_pub_;
    int debug_path_id_;
    
    // For Trajectory Rollout debugging
    bool VISUALIZATION_;
    bool DEBUG_ON_;
    int line_id_;
    int chosen_path_line_id_;
    bool traj_drawn_;
    
    ros::Publisher all_path_pub_;
    ros::Publisher chosen_path_pub_;
    ros::Publisher start_line_pub_;

=======
=======
>>>>>>> parent of 2d82fc5... All code changes from competition. This code is functional.
>>>>>>> f441e361b6fa7c6003fc3961b287c77f441401b0
};

#endif  // PATHPLANNER_H
