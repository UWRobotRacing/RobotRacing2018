// Copyright [2015] University of Waterloo Robotics Team
/***********************************************************************************
  Robot Racer Motion Primitive Planner
  Date:11/07/14

  This function takes in the drivability map, and publishes steering command to the 
  arduino, It assumes a kinematic vehicle model. It also publishes velocity command 
  to the Long. controller.

      ** If enable is off, then steering stops publishing anything! Velocity publishes zero
      ** If distance to the object < MIN_DISTANCE_, then the vehicle stops
      ** Always assumes initial vehicle position is at (0,0) = mid bottom of the image

  Author: Sirui Song, Jungwook Lee

************************************************************************************/

#include <iostream>
#include <algorithm>
#include <vector>
#include <limits> // for infinity

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <PathPlanner.h>

PathPlanner::PathPlanner()
{
  // Set bottom middle as trajectory rollout start location
  getParam();

  // Initialize visualization members
  line_id_ = 0;
  chosen_path_line_id_ = 1;
  debug_path_id_ = 2;

  start_line_detected_ = false;

  // Initialize start locations (might want to change this based on application)
  X_START_ = 0;
  Y_START_ = -1*((map_H_/2)*resolution_);   //**  y=-2

  trajectory_index_.resize(NUM_PATHS_);
  for (int i = 0; i < NUM_PATHS_; ++i)
  {
    trajectory_index_[i].resize(TRAJECTORY_STEPS_);
  }

  angles_weight_.resize(NUM_PATHS_);
  distance_.resize(NUM_PATHS_, std::vector<double>( TRAJECTORY_STEPS_, 0.0));
  test_markers_.resize(NUM_PATHS_);

  // Load Path Indexes
  initializePaths();

  map_sub_ = nh_.subscribe("/map", 1, &PathPlanner::callBack, this);
  cmd_pub_ = nh_.advertise<std_msgs::Float32>("/PathPlanner/vel_level", 1, true);
  steer_pub_ = nh_.advertise<std_msgs::Float32>("/PathPlanner/steer_cmd", 1, true);
  start_line_pub_ = nh_.advertise<std_msgs::Bool>("/PathPlanner/start_line_detected",1,true);

  if (VISUALIZATION_)
  {
    all_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/all_paths", 1, true);
    chosen_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/chosen_path", 1, true);
    test_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/debug_paths", 1, true);
    drawLine(all_path_pub_, all_paths, line_id_,55.0, 0.0, 0.0, 1, 1.0);
  }
}

// Get Parameters from yaml or launch file
void PathPlanner::getParam()
{
  nh_.param<int>("TrajRoll/MAP_WIDTH", map_W_, 600);
  nh_.param<int>("TrajRoll/MAP_HEIGHT", map_H_, 400);
  nh_.param<double>("TrajRoll/RESOLUTION", resolution_, 0.01);

  nh_.param<double>("TrajRoll/PLANNER_VELOCITY", PLANNER_VELOCITY_, 2.0);
  // Trajectory Rollout Parameters
  nh_.param<int>("TrajRoll/NUM_PATHS", NUM_PATHS_, 21);
  if (!(NUM_PATHS_ % 2 > 0))
  {
    ROS_WARN("PathPlanner: NUM_PATHS is not odd. Using 21 as default.");
    NUM_PATHS_ = 21;
  }
  nh_.param<int>("TrajRoll/TRAJECTORY_STEPS", TRAJECTORY_STEPS_, 100);
  nh_.param<double>("TrajRoll/dt", dt_, 0.05);  //**should be calculated as per the frequency rate given in main.cpp
  nh_.param<int>("TrajRoll/OBS_THRESHOLD", OBS_THRESHOLD_, 60);
  nh_.param<double>("TrajRoll/MIN_STOPPING_DIST", MIN_STOPPING_DIST_, 0.2);

  nh_.param<double>("TrajRoll/ANGLE_WEIGHT", ANGLE_WEIGHT_, 1);
  nh_.param<double>("TrajRoll/DISTANCE_WEIGHT", DISTANCE_WEIGHT_, 1.9);
  nh_.param<double>("TrajRoll/ANGLE_WEIGHT_DIFF", ANGLE_WEIGHT_DIFF_, 1);

  nh_.param<double>("TrajRoll/UX_DESIRED", UX_DESIRED_, 2.0);
  nh_.param<double>("TrajRoll/MIN_ACCELERATION_ANGLE", MIN_ACCELERATION_ANGLE_, 2.0);
  nh_.param<double>("TrajRoll/STRAIGHT_SPEED", STRAIGHT_SPEED_, 0.5);

  nh_.param<double>("TrajRoll/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE_, 0.2);   // 0.2 rad ~ 11 degree; 0.8 rad ~ 45 degree;
  nh_.param<double>("TrajRoll/CAR_WIDTH", CAR_WIDTH_, 0.1);
  nh_.param<double>("TrajRoll/DIST_COST_FACTOR", DIST_COST_FACTOR_, 1.0);
  nh_.param<double>("TrajRoll/MIN_OFFSET_DIST", min_offset_dist_, 0.1);

  nh_.param<double>("TrajRoll/HORIZONTAL_LINE_RATIO", HORIZONTAL_LINE_RATIO_, 0.02);

  nh_.param<bool>("TrajRoll/VISUALIZATION", VISUALIZATION_, true);
  nh_.param<bool>("TrajRoll/DEBUG_ON", DEBUG_ON_, false);
  nh_.param<bool>("TrajRoll/Check_Traffic", check_traffic_, true);
}

void PathPlanner::callBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("PATH PLANNER: callBack: Started");
<<<<<<< HEAD
=======

>>>>>>> 909a190b75bd2c012ec02c88dbf3fb51b1f6f34d
  // Exit if not enabled
  if (check_traffic_)
  {
    if (!enableCatcher_.isSignalled())
    {
      return;
      ROS_INFO("PATH PLANNER: callBack: ERROR: EXITING");
    }
  }
    // if (DEBUG_ON_)
  //   ROS_INFO("PATH PLANNER: callBack: Start location (%d, %d)", X_START_,Y_START_);

 
   if (DEBUG_ON_) {ROS_INFO("PATH PLANNER: callBack: Start location (%d, %d)", X_START_,Y_START_);}

  // if (DEBUG_ON_)
  //   ROS_INFO("PATH PLANNER: callBack: Start location (%d, %d)", X_START_,Y_START_);

  map_ = msg;
  generatePaths();
  ROS_INFO("PATH PLANNER: callBack: Called Successfully");
}

/*  
 *  Creates the Vector of Vectors that describes the trajectory paths 
 */
void PathPlanner::initializePaths()
{
  // Generate the angles
  angles_ =
    generateAngles(NUM_PATHS_, MAX_STEERING_ANGLE_);

  trajectory_points.resize(NUM_PATHS_);

  // Generate the vectors
  for (int i = 0; i < NUM_PATHS_; i++)
  {
    // if (DEBUG_ON_)
    //   ROS_INFO("PATH PLANNER: initializePaths: Angle #%d, %f rad", i, angles[i]);

    // Trajectory Variables
    double x = X_START_;
    double y = Y_START_;
    double cur_heading = 0.0;   //** theta from state matrix

    double heading = angles_[i];
    double distance = 0.0;

    for (int j = 0; j < TRAJECTORY_STEPS_; j++ )
    {
      x += PLANNER_VELOCITY_*sin(-cur_heading)*dt_; //** robot dx as calculated by motion model
      y += PLANNER_VELOCITY_*cos(-cur_heading)*dt_; //** they have included -theta bcoz angles range from -M to +M therefore imagine robot movement
      cur_heading += (heading * PLANNER_VELOCITY_ * dt_);   //** it seems wrong; it should be theta + d_theta (they have not included the wheel radius)
      distance += (sqrt(pow(PLANNER_VELOCITY_*sin(cur_heading)*dt_, 2) + pow(PLANNER_VELOCITY_*cos(cur_heading)*dt_, 2)));
      distance_[i][j] = distance;

      double x0 = x + (CAR_WIDTH_) * cos(cur_heading);
      double y0 = y + (CAR_WIDTH_) * sin(cur_heading);
      double x1 = x - (CAR_WIDTH_) * cos(cur_heading);
      double y1 = y - (CAR_WIDTH_) * sin(cur_heading);

      if (VISUALIZATION_)
      {
        pushPoints(test_markers_[i], x0, y0);
        pushPoints(test_markers_[i], x1, y1);
      }

      std::vector<geometry_msgs::Point> new_point_list = rayTrace(x0, y0, x1, y1);
      for (int k = 0; k < new_point_list.size(); k++)
      {
        int tmp_index = xyToMapIndex(new_point_list[k].x,new_point_list[k].y);
        if (VISUALIZATION_)
        {
          pushPoints(test_markers_[i], new_point_list[k].x, new_point_list[k].y);
        }
        trajectory_index_[i][j].push_back(tmp_index);
      }

      if (VISUALIZATION_)
      {
        // Visualization Variables
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        all_paths.points.push_back(p);
        trajectory_points[i].points.push_back(p);
      }
    }
  }
  ROS_INFO("PATH PLANNER: initializePaths: Finished Initializing Paths");
}

std::vector<geometry_msgs::Point> PathPlanner::rayTrace(double x0, double y0, double x1, double y1)
{
  // Bresenham's line algorithm
  float scale_factor = 100.0;
  x0 *= scale_factor;
  x1 *= scale_factor;
  y0 *= scale_factor;
  y1 *= scale_factor;
  std::vector<geometry_msgs::Point> new_list;

  double dx = fabs(x1 - x0);
  double dy = fabs(y1 - y0);

  int x = int(floor(x0));
  int y = int(floor(y0));

  int n = 1;
  int x_inc, y_inc;
  double error;

  if (dx == 0)
  {
    x_inc = 0;
    error = std::numeric_limits<double>::infinity();
  }
  else if (x1 > x0)
  {
    x_inc = 1;
    n += int(floor(x1)) - x;
    error = (floor(x0) + 1 - x0) * dy;
  }
  else
  {
    x_inc = -1;
    n += x - int(floor(x1));
    error = (x0 - floor(x0)) * dy;
  }

  if (dy == 0)
  {
    y_inc = 0;
    error -= std::numeric_limits<double>::infinity();
  }
  else if (y1 > y0)
  {
    y_inc = 1;
    n += int(floor(y1)) - y;
    error -= (floor(y0) + 1 - y0) * dx;
  }
  else
  {
    y_inc = -1;
    n += y - int(floor(y1));
    error -= (y0 - floor(y0)) * dx;
  }

  for (; n > 0; --n)
  {
    geometry_msgs::Point pt;
    pt.x = x/scale_factor;
    pt.y = y/scale_factor;
    new_list.push_back(pt);

    if (error > 0)
    {
      y += y_inc;
      error -= dx;
    }
    else
    {
      x += x_inc;
      error += dy;
    }
  }
  return new_list;
}

void PathPlanner::pushPoints(visualization_msgs::Marker& marker, double x, double y)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  marker.points.push_back(p);
}


void PathPlanner::generatePaths()
{
    ROS_INFO("PATH PLANNER: generatePaths: Called");
  // start to record time
  if (!start_recorded_)
  {
    start_time_ = ros::Time::now();
    start_recorded_ = true;
  }

  if (map_ == NULL)
  {
    ROS_INFO("PATH PLANNER: generatePaths: Null Pointer");
  }

  double distance = 0;
  double velocity = UX_DESIRED_;
  int chosen_angle_index = 0;
  double norm_dist_cost;
  double norm_angle_cost;
  double highest_value;
  double cost;
  int index_of_longest_path;
  float distance_of_chosen_path;
  float angle_of_chosen_path;

  start_line_detected_ = false;

  for (int i = 0; i < NUM_PATHS_; i++)
  {
    int index_of_path = checkLength(i);

    distance = distance_[i][index_of_path];
    norm_dist_cost = DIST_COST_FACTOR_*distance/(PLANNER_VELOCITY_*TRAJECTORY_STEPS_*dt_);
    norm_angle_cost = angles_weight_[i];
    cost = norm_dist_cost + norm_angle_cost;

    if (DEBUG_ON_)
        {
        ROS_INFO("PATH PLANNER: generatePaths: Longest temp_distance at index %d = %f", i, distance);
        ROS_INFO("PATH PLANNER: generatePaths: norm_dist_cost %d = %f", i, norm_dist_cost);
        ROS_INFO("PATH PLANNER: generatePaths: norm_angle_cost %d = %f", i, norm_angle_cost);
        ROS_INFO("PATH PLANNER: generatePaths: cost %d = %f", i, cost);
    }

    if (i == 0)
    {
      highest_value = norm_dist_cost + norm_angle_cost;
      index_of_longest_path = 0;    //should = index_of_path
    }

    if (highest_value < cost)
    { 
      highest_value = cost;
      chosen_angle_index = i;
      angle_of_chosen_path = angles_[i];
      distance_of_chosen_path = distance;
      index_of_longest_path = index_of_path;
    }
  }

  if (DEBUG_ON_) {
     ROS_INFO("PATH PLANNER: generatePaths: Chosen Line = %d", chosen_angle_index);
     ROS_INFO("PATH PLANNER: generatePaths: Longest Distance = %f", distance);
     ROS_INFO("PATH PLANNER: generatePaths: Output Angle = %f", -1*angles_[chosen_angle_index]);
  }

  std_msgs::Bool start_line_val;
  start_line_val.data = start_line_detected_;
  start_line_pub_.publish(start_line_val);

  // Steering command
  cmdVelMsg_.data = UX_DESIRED_;
  if (distance_of_chosen_path <= MIN_STOPPING_DIST_)
  {
    cmdVelMsg_.data = 0.0;
  } 
  if ( fabs(angle_of_chosen_path) <= MIN_ACCELERATION_ANGLE_)
  {
    cmdVelMsg_.data = STRAIGHT_SPEED_;
  }

  // Stopping Strategy
  if (DRAG_MODE_)
  {
    ros::Duration diff = ros::Time::now() - start_time_;
    // ROS_INFO("Current time diff : %d", diff.sec);
    // ROS_INFO("Current time diff nano: %d", diff.nsec);
    if (diff > drag_duration_)  //** for stopping the car after a fixed time if in case it doesnot detect start_line_
    {
      cmdVelMsg_.data = 0.0;
    }
  }
  else
  {
    if (start_line_detected_)
    {
      cmdVelMsg_.data = 0.0;
    }
  }
  
  ROS_INFO("PATH PLANNER: cmdVelMsg_ is  %f", cmdVelMsg_);

  cmd_pub_.publish(cmdVelMsg_);

  cmdAngleMsg_.data = angles_[chosen_angle_index];
  steer_pub_.publish(cmdAngleMsg_);

  if (VISUALIZATION_ == true)
  {
    drawChosenLine(chosen_path_pub_, trajectory_points[NUM_PATHS_-1-chosen_angle_index], index_of_longest_path, chosen_path_line_id_,0.0, 50.0, 0.0, CAR_WIDTH_*2, 1);
    drawLine(test_pub_, test_markers_[NUM_PATHS_-1-chosen_angle_index], debug_path_id_,0.0, 0.0, 50.0, 1, 1.0);
  }
}

void PathPlanner::drawLine(ros::Publisher& pub, visualization_msgs::Marker& points, int id, int color_R, int color_G, int color_B, float scale, float alpha)
{
  points.header.frame_id = "/map";
  points.header.stamp = ros::Time::now(); 
  points.ns = "Path Points";
  points.action = visualization_msgs::Marker::ADD;
  points.id = id;
  //ROS_INFO("PATH PLANNER: drawLine: Drawing Line #%d", line_id_);
  points.type = visualization_msgs::Marker::LINE_STRIP;
  // Each curve must have a unique id or you will overwrite an old ones
  points.scale.x = scale*0.01;
  //points.scale.y = 0.001;
  points.color.r = color_R;
  points.color.g = color_G;
  points.color.b = color_B; 
  points.color.a = alpha;
  pub.publish(points);
}

void PathPlanner::drawChosenLine(ros::Publisher& pub, visualization_msgs::Marker& points, int index, int id, int color_R, int color_G, int color_B, float scale, float alpha)
{
  points.header.frame_id = "/map";
  points.header.stamp = ros::Time::now(); 
  points.ns = "Path Points";
  points.action = visualization_msgs::Marker::ADD;
  points.id = id;
  //ROS_INFO("PATH PLANNER: drawLine: Drawing Line #%d", line_id_);
  points.type = visualization_msgs::Marker::POINTS;
  // Each curve must have a unique id or you will overwrite an old ones
  points.scale.x = scale;
  points.scale.y = 0.01;
  points.color.r = color_R;
  points.color.g = color_G;
  points.color.b = color_B; 
  points.color.a = alpha;
  std::vector<geometry_msgs::Point>::const_iterator first = points.points.begin();
  std::vector<geometry_msgs::Point>::const_iterator last = points.points.begin()+index;
  std::vector<geometry_msgs::Point> newPoints (first, last);
  points.points = newPoints;
  pub.publish(points);
}

//  CheckMap function: determines whether or not the cell is occupied
//  This operates in image-inertial frame (units are SI x, y, in m)
bool PathPlanner::checkMap(double x, double y)  //** haven't used this function.. instead used checkLength() function;
{
  double grid_x = round(x/resolution_);
  double grid_y = round(y/resolution_);
  int Map_index = xyToMapIndex(x,y);

  if (abs(grid_x) <= map_W_/2 && abs(grid_y) <= map_H_/2)
  {
    if (map_ == NULL)
    {
      ROS_INFO("PATH PLANNER: checkMap: Null Pointer");
    }

    if (DEBUG_ON_)
    {
      ROS_INFO("PATH PLANNER: checkMap: Index: %d ", Map_index);
      ROS_INFO("PATH PLANNER: checkMap: Value: %d ", map_->data[Map_index]);
    }

    if (map_->data[Map_index] > 0)
    {
      if (DEBUG_ON_)
        ROS_INFO("PATH PLANNER: checkMap: Something hitting the car. Exit.");
      return false;
    }

    if (DEBUG_ON_)
      ROS_INFO("PATH PLANNER: checkMap: Nothing is in the way. Next Iteration.");
    return true;
  }
  if (DEBUG_ON_)
    ROS_INFO("PATH PLANNER: checkMap: Checking outer bounds. Exit.");
  return false;
}

// Generate path angles (number of lines, bias toward centre, steering resolution)
std::vector<double> PathPlanner::generateAngles(int numLines, double MAX_STEER)
{
  if (numLines < 1)
    numLines = 10;

  std::vector<double> angles(numLines);

  for (int i = 0; i < numLines; i++)
  {
    angles[i] = -MAX_STEER + (MAX_STEER*2/(numLines-1)*i);  //** angle range -max_steer to +max_steer divided equally in numPaths
    angles_weight_[i] = 1.0-(abs(i-(numLines/2)))*(0.01/numLines);  //**gives maximum weight (1.0) at the center path i.e. straight path. and with increase in angle from center, angle weight decreases.
    // if (DEBUG_ON_){
    //   ROS_INFO("PathPlanner: generateAngles: angle at %d : %f", i, angles[i]);
    //   ROS_INFO("PathPlanner: generateAngles: angles_weight at %d : %f", i, angles_weight_[i]);
    // }
  }
  return angles;
}

// Check trajectory_length (steering angle, velocity) check max length with collision.
int PathPlanner::checkLength(int angle_index)
{
  int start_spine_index = 0;
  int index_of_longest_path = 0;    //** returns the no. of TRAJECTORY_STEPS_ before obstacle
  int obstacle_count = 0;

  // Iterate through spine from bottom
  for (int i = 0; i < TRAJECTORY_STEPS_; i++)
  {
    // Check for minimum obstacle distance to ignore
    if (distance_[angle_index][i] >= min_offset_dist_)
    {
      int spine_length = trajectory_index_[angle_index][i].size();  //** delta displacement in time dt
      obstacle_count = 0;
      // Iterate through the sides in each spine index
      for (int j = 0; j < spine_length; j++)
      {
          // ROS_INFO("PATH PLANNER: checkLength: Cur Line: %d, iteration %d", angle_index, i);
          // ROS_INFO("PATH PLANNER: checkLength: Cur Spine Index : %d", i);
          // ROS_INFO("PATH PLANNER: checkLength: Cur Side Index : %d", j);
          // ROS_INFO("PATH PLANNER: checkLength: Cur Side : %d", trajectory_index_[angle_index][i][j]);

        // Count obstacles in the spine of the paths
        if (isOBS(trajectory_index_[angle_index][i][j]))    //** if obstacle detected
        {
          // ROS_INFO("PATH PLANNER: checkLength: Line i = : %d", angle_index);
          // ROS_INFO("PATH PLANNER: checkLength: Spine index at : %d", i);
          // ROS_INFO("PATH PLANNER: checkLength: Side index at : %d", j);
          obstacle_count += 1;
        }
      }
      //** no idea below this
      
      if ((obstacle_count) >= (spine_length*HORIZONTAL_LINE_RATIO_)) 
      {
        // int size = trajectory_index_[angle_index][i].size();
        // ROS_INFO("PATH PLANNER: checkLength: Horizontal Count %d", size);
        // ROS_INFO("PATH PLANNER: checkLength: Horizontal Detected at line %d", angle_index);
        start_line_detected_ = true;
        //start_line_pub.publish();
      }
      else if (obstacle_count > 0)
      {
        index_of_longest_path = i;
        return i;
      }
      // Check for obstacle and exit early
    }
    index_of_longest_path = i;
  }
  return index_of_longest_path;
}

bool PathPlanner::isOBS(int index)
{
  //ROS_INFO("PATH PLANNER: isOBS: Checking: %d", index);
  if (map_->data[index] != OBS_THRESHOLD_)      //** should be > OBS_THRESHOLD_
  {
    return true;
  }
  return false;
}

/* Not Used */
bool PathPlanner::checkBox(int Map_index)
{
  ROS_INFO("PATH PLANNER: checkBox: checking start %d", Map_index);
  int car_pixel_size = CAR_WIDTH_/resolution_;
  for (int j = round(-car_pixel_size/2); j < round(car_pixel_size/2); j++)
  {
    for (int i = round(-car_pixel_size/2); i < round(car_pixel_size/2); i++)
    {
      int cur_index = Map_index + ((i-1)+((j-1)*(map_W_)));
      if (cur_index > 0 && cur_index <= (map_W_-1)+((map_H_-1)*map_W_)) 
      {
      ROS_INFO("PATH PLANNER: checkBox: checking at %d", cur_index);
        if (map_->data[Map_index] != 0)
        {
          ROS_INFO("PATH PLANNER: checkBox: something at %d", Map_index);
          return false; 
        }
      }
    }
  }
  return true;
}

// Convert x,y in car frame to occupancy grid index
int PathPlanner::xyToMapIndex(double x, double y)  {
    // Convert [m] to [pixel] in car coordinate. (origin at bottom middle of image)
  double grid_x = round(x/resolution_);
  double grid_y = round(y/resolution_);

  // Convert [pixel] in car coordinate to [pixel] in image frame
  double map_x  = grid_x + round( map_W_/2 );
  double map_y  = -1*grid_y + round( map_H_/2 );

  // Convert [pixel] in image frame to index in occupancy grid format
  int Map_index = (map_x-1)+(map_y-1)*map_W_;   //**map index starts from 0,0 from top left
  return Map_index;
}
