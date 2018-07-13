/** @file path_planner.cpp
 *  Robot Racer Motion PathPlanner
 *
 *  Topics Subscribed:
 *    /map
 *
 *  Topics Published:
 *    /PathPlanner/pp_cmd          --used by arduino and simulation
 *
 *  motion model used: 4 wheeled front steered car model
 *
 * Always assumes initial vehicle position is at (0,0) = mid bottom of the image
 *
 *  @author Ajay Kumar Singh
 *  @author Toni Ogunmade
 */
#include <path_planner.h>

/** @brief initializes the path planner object
 */
PathPlanner::PathPlanner()
{
  // Setting initial variables and parameters
  Init();

  //initializing angles and corresponding weights
  angles_and_weights = GetAnglesAndWeights(MAX_STEERING_ANGLE_, NUM_PATHS_);

  //VISUALIZATION Pub
  if(VISUALIZATION_)
  {
    all_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/all_paths", 1, true);
    selected_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/selected_path", 1, true);
    rayTrace_pub_ = nh_.advertise<visualization_msgs::Marker>("/Visualization/rayTrace", 1, true);
    X_axis_pub = nh_.advertise<visualization_msgs::Marker>("/Visualization/X_axis", 1, true);
    Y_axis_pub = nh_.advertise<visualization_msgs::Marker>("/Visualization/Y_axis", 1, true);
  }
  //initializing all possible paths of trajectory rollout
  GenerateIdealPaths();

  //ros topics init
  //Normal Pub and Subscriber
  map_sub_ = nh_.subscribe(rr_processed_topics::fused_map, 1, &PathPlanner::ProcessMap, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(rr_cmd_topics::vel_cmd, 1, true);
}

/** @brief get the parameters and initialize the member variables
 */
void PathPlanner::Init()
{
  GetParams();
  // debug parameters
  all_path_marker_id_ = 0; //id for the selected path marker
  selected_path_marker_id_ = 1;
  trajectory_marker_rayTrace_id_ = 4;
  trajectory_marker_rayTrace_.resize(NUM_PATHS_);
  X_axis_marker_id = 2;
  Y_axis_marker_id = 3;
  geometry_msgs::Point p1, p2;

  for ( int i = 0; i < 200; i++)
  {
    p1.x = i * resolution_;
    p1.y = 0;
    p2.x = 0;
    p2.y = i * resolution_;
    X_axis_marker_points.points.push_back(p1);
    Y_axis_marker_points.points.push_back(p2);
  }

  trajectory_marker_vector_.resize(NUM_PATHS_);

  //resizing the path variables
  angles_and_weights.resize(NUM_PATHS_, std::vector<double>(2, 0.0));
  path_distance.resize(NUM_PATHS_, std::vector<double>(TRAJECTORY_STEPS_, 0.0));
  trajectory.resize(NUM_PATHS_);

  for(int i=0;i<NUM_PATHS_;i++)
  {
    trajectory[i].resize(TRAJECTORY_STEPS_);
  }

  if(DEBUG_ON_)
  {
    ROS_INFO("PATH PLANNER: Init() : Finished Initializing");
  }
}

/** @brief save the values of the ros parameters used for path generation
 */
void PathPlanner::GetParams()
{
  nh_.param<int>("TrajRoll/MAP_WIDTH", map_W_, 600);
  nh_.param<int>("TrajRoll/MAP_HEIGHT", map_H_, 400);
  nh_.param<double>("TrajRoll/RESOLUTION", resolution_, 0.05);
  nh_.param<int>("TrajRoll/X_START", X_START_, 0);
  nh_.param<int>("TrajRoll/Y_START", Y_START_, 0);

  // Trajectory Rollout Parameters
  nh_.param<int>("TrajRoll/NUM_PATHS", NUM_PATHS_, 21);
  if ((NUM_PATHS_ % 2) == 0)
  {
    ROS_WARN("PathPlanner: NUM_PATHS is not odd. Using 21 as default.");
    NUM_PATHS_ = 21;
  }
  nh_.param<int>("TrajRoll/TRAJECTORY_STEPS", TRAJECTORY_STEPS_, 100);
  nh_.param<double>("TrajRoll/MIN_STOPPING_DIST", MIN_STOPPING_DIST_, 0.3);
  nh_.param<double>("TrajRoll/STOPPING_FACTOR", STOPPING_FACTOR_, 0.15);

  nh_.param<double>("TrajRoll/STRAIGHT_SPEED", STRAIGHT_SPEED_, 0.5);

  nh_.param<double>("TrajRoll/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE_, 0.8);   //** 0.2 rad ~ 11 degree; 0.8 rad ~ 45 degree;
  nh_.param<double>("TrajRoll/CAR_WIDTH", CAR_WIDTH_, 0.27);
  nh_.param<double>("TrajRoll/DIST_REWARD_FACTOR", DIST_REWARD_FACTOR_, 1.0);
  nh_.param<double>("TrajRoll/MIN_OFFSET_DIST", min_offset_dist_, 0.1);

  nh_.param<bool>("TrajRoll/VISUALIZATION", VISUALIZATION_, true);
  nh_.param<bool>("TrajRoll/DEBUG_ON", DEBUG_ON_, false);
  nh_.param<double>("simulated_odometry/wheel_to_wheel_dist", wheel_to_wheel_dist_, 0.335);

  //ROS_INFO("<<<<<<<< Loaded Parameters >>>>>>>>");
}

/** @brief get a list of weight for each of the angles under consideration
 *  @param max_angle the largest angle under consideration
 *  @param num_paths total number of paths to be rolled out
 *  @return a vector of weights for each of the angles
 */
std::vector< std::vector <double> > PathPlanner::GetAnglesAndWeights(double max_angle, int num_paths)
{
  std::vector< std::vector <double> > angle_n_weights(num_paths, std::vector<double> (2, 0.0));
  for(int i=0;i<num_paths;i++) 
  {
    // angle range -max_angle to +max_angle divided equally in num_paths
    angle_n_weights[i][0] = (max_angle*2/(num_paths-1)*i) - max_angle;
    //gives maximum weight(1.0) at center path i.e. straight path. & with increase in angle from center, angle weight decreases.
    angle_n_weights[i][1] = 1.0-(abs(i-(num_paths/2)))*(0.01/num_paths);
  }
  return angle_n_weights;
}

/**
 * @brief Get the Velocity Magnitude object
 *
 * @param input the twist message
 * @return double the velocity magnitude
 */
inline double GetVelocityMagnitude(geometry_msgs::Twist input)
{
  return sqrt(pow(input.linear.x, 2) + pow(input.linear.y, 2));
}

/** @brief generate paths based on the steering angles in angles_and_weights
 *
 *  theta is the steering angle
 *  the steering angles are used to generate a list of point that is then
 *  saved for later use
 *  if VISUALIZATION_ is set it will be displayed
 */
void PathPlanner::GenerateIdealPaths()
{
  for(int i=0;i<NUM_PATHS_;i++)
  {
    //initial state vector values of car
    double x = X_START_;
    double y = Y_START_;
    double theta = 0.0;
    double dist = 0.0;
    double dt = 0.05;
    double planning_speed = 1;

    for(int j = 0; j < TRAJECTORY_STEPS_; j++)
    {
      x += planning_speed * sin(-theta) * dt; //** robot dx as calculated by motion model
      y += planning_speed * cos(-theta) * dt; //** -theta flips the Y axis. theta is measured around Y_axis
      theta += angles_and_weights[i][0] * planning_speed * dt;
      dist += sqrt(pow(planning_speed * sin(theta) * dt_, 2) + pow(planning_speed * cos(theta) * dt, 2));
      path_distance[i][j] = dist;
      //compensating rectangluar car around x and y
      double x0 = x + (CAR_WIDTH_) * cos(theta);
      double y0 = y + (CAR_WIDTH_) * sin(theta);
      double x1 = x - (CAR_WIDTH_) * cos(theta);
      double y1 = y - (CAR_WIDTH_) * sin(theta);

      std::vector<geometry_msgs::Point> new_point_list = rayTrace(x0, y0, x1, y1);
      for (int k = 0; k < new_point_list.size(); k++)
      {
        int tmp_index = xyToMapIndex(new_point_list[k].x, new_point_list[k].y);
        trajectory[i][j].push_back(tmp_index);
      }
      if (VISUALIZATION_)
      {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        trajectory_marker_vector_[i].points.push_back(p);
        trajectory_points_.points.push_back(p);
        for(std::vector<geometry_msgs::Point>::iterator it = new_point_list.begin(); it<new_point_list.end(); it++)
        {
          trajectory_marker_rayTrace_[i].points.push_back(*it);
        }
      }
    }
  }
  if(!all_path_pub_)
  {
    ROS_WARN("Invalid publisher");
  }
  if(VISUALIZATION_)
  {
    DrawPath(all_path_pub_, trajectory_points_, all_path_marker_id_, -1, 50, 0, 0, 0.01, 1.0);
    DrawPath(X_axis_pub, X_axis_marker_points, X_axis_marker_id, -1, 100, 0, 0, 0.05, 1.0);
    DrawPath(Y_axis_pub, Y_axis_marker_points, Y_axis_marker_id, -1, 0, 0, 100, 0.05, 1.0);
  }
}

/** @brief serves as the callback for the occupancy grid containing the lanes and obstacles
 *  @param msg is a pointer to the OccupancyGrid object
 */
void PathPlanner::ProcessMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if(DEBUG_ON_) 
  {
    ROS_INFO("ProcessMap() : Called: started");
  }
  
  if (DEBUG_ON_)
  {
    ROS_INFO("PATH PLANNER: ProcessMap: Start location (%d, %d)", X_START_,Y_START_);
  }
  map_ = msg;
  GenerateRealPaths();
}

/** @brief score each path based on it's obstacle free length, 
 */
void PathPlanner::GenerateRealPaths()
{
  //generating realtime path, avoiding obstacles
  if(DEBUG_ON_)
  {
    ROS_INFO("PATH PLANNER: generating RealPaths");
  }
  if(map_ == NULL)
  {
    ROS_WARN("PATH PLANNER: GenerateRealPaths(): Null Pointer");
  }

  //compute weights, select optimum path.
  double dist = 0.0;
  double dist_reward = 0.0;
  double angle_reward = 0.0;
  double reward = 0.0;
  double highest_reward = 0.0;
  int index_of_longest_path = 0;
  int selected_path_index = 0;
  double selected_path_angle;
  double selected_path_distance;

  for (int i = 0; i < NUM_PATHS_; i++)
  {
    int index_on_path = CheckLength(i);
    dist = path_distance[i][index_on_path];
    dist_reward = DIST_REWARD_FACTOR_ * dist;
    angle_reward = angles_and_weights[i][1];
    reward = dist_reward + angle_reward;
    if (i == 0)
    {
      highest_reward = reward;
      index_of_longest_path = index_on_path;
      selected_path_index = i;
    }
    if (highest_reward < reward)
    {
      highest_reward = reward;
      selected_path_index = i;
      index_of_longest_path = index_on_path;
      selected_path_angle = angles_and_weights[i][0];
      selected_path_distance = dist;
    }
  }

//  ROS_INFO("PATH PLANNER: GenerateRealPaths: Selected path = %d", selected_path_index);
//  ROS_INFO("PATH PLANNER: GenerateRealPaths: Longest Distance = %f", selected_path_distance);
//  ROS_INFO("PATH PLANNER: GenerateRealPaths: selected Angle = %f", selected_path_angle);

  selected_path_index = NUM_PATHS_ - selected_path_index - 1;
  double wheel_speed = Velocity(selected_path_distance, selected_path_angle);

  double lf = wheel_to_wheel_dist_ / 2.0;
  double lr = lf;
  double beta = atan2(lr * tan(selected_path_angle), lf + lr);

  //since we don't have access to the current orientation of the car use 45 degrees as a placeholder
  //TODO(oluwatoni) change this?
  vel_cmd_.linear.x = wheel_speed * cos(0.7071 + beta);
  vel_cmd_.linear.y = wheel_speed * sin(0.7071 + beta);
  vel_cmd_.angular.z = (wheel_speed / lr) * -sin(beta);
  //Publish vel_level, steer_cmd,
  cmd_pub_.publish(vel_cmd_);
  //Publish for VISUALIZATION_: selected_path based on index_of_longest_path
  if(VISUALIZATION_)
  {
    DrawPath(selected_path_pub_, trajectory_marker_vector_[selected_path_index], selected_path_marker_id_, index_of_longest_path, 0, 0, 50, 0.05, 1);
    DrawPath(rayTrace_pub_, trajectory_marker_rayTrace_[selected_path_index], trajectory_marker_rayTrace_id_, -1, 0, 50, 50, 0.01, 0.5);
  }
}

/** @brief returns the obstacle free length of the supplied path
 *  @param angle_index the path to be considered
 *  @return the index of the farthest point along the path
 */
int PathPlanner::CheckLength(int angle_index)
{
  for (int j = 0; j < TRAJECTORY_STEPS_; j++)
  {
    if (path_distance[angle_index][j] >= min_offset_dist_)
    {
      int spine_length = trajectory[angle_index][j].size();
      for (int k = 0; k < spine_length; k++)
      {
        if (IsCellOccupied(trajectory[angle_index][j][k]))
        {
          //obstacle detected in the cell
          return j;
        }
      }
    }
  }
  return TRAJECTORY_STEPS_ - 1;
}

/** @brief returns if a cell is occupied or not
 *  @param index the index of that cell on the OccupancyGrid
 *  @return a boolean representing the state of the cell
 */
bool PathPlanner::IsCellOccupied(int index)
{
  if(map_-> data[index] != 0)
  {
    return true;
  }
  return false;
}

/**
 * @brief draw the selected path over the OccupancyGrid
 * 
 * @param pub the markerArray publisher
 * @param points the points along the selected path
 * @param id the id
 * @param index the index of the last point on the path
 * @param R the red value of the markers
 * @param G the green value of the markers 
 * @param B the blue value of the markers 
 * @param scale the scale of the markers
 * @param alpha the transparency of the markers
 */
void PathPlanner::DrawPath(ros::Publisher& pub, visualization_msgs::Marker& points,int id, int index, int R, int G, int B, float scale, float alpha) {
  points.header.frame_id = "/base_link";
  points.header.stamp = ros::Time::now();
  points.ns = "Path Points";
  points.action = visualization_msgs::Marker::ADD;
  points.id = id;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = scale;
  points.scale.y = scale;
  points.color.r = R;
  points.color.g = G;
  points.color.b = B;
  points.color.a = alpha;

  if(index >= 0)
  {
    std::vector<geometry_msgs::Point>::const_iterator first = points.points.begin();
    std::vector<geometry_msgs::Point>::const_iterator last = points.points.begin()+index;
    std::vector<geometry_msgs::Point> newPoints (first, last);
    points.points = newPoints;
  }
  pub.publish(points);
}

/**
 * @brief generate a list of points connecting the coordinates provided
 *  
 * Works based on Bresenham's line algorithm
 * 
 * @param x0 the x comp of the originating point
 * @param y0 the y comp of the originating point
 * @param x1 the x comp of the terminating point
 * @param y1 the y comp of the terminating point
 * @return std::vector<geometry_msgs::Point> the list of points
 */
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

  if(DEBUG_ON_)
  {
    ROS_INFO("rayTrace() : mid :5a");
  }

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

  if(DEBUG_ON_)
  {
    ROS_INFO("rayTrace() : mid :5b");
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
    //if(DEBUG_ON_) {ROS_INFO("rayTrace() : just before new_list value set :5d");}
    //if(DEBUG_ON_) {ROS_INFO("rayTrace() : value of n=%d, error=%f", n, error);}
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

/**
 * @brief sets the velocity of the car based on the length and curvature of the path selected
 * 
 * @param dist the length of the path
 * @param steer the steering angle selected
 * @return std_msgs::Float32 the goal velocity 
 */
double PathPlanner::Velocity(double dist, double steer)
{
  double vel = 0;
  if((dist <= StopDistFromVel(vel_cmd_)) || (dist <=MIN_STOPPING_DIST_))
  {
    vel = 0.0;
  }
  else
  {
    //turning radius break point at
    vel = std::min(STRAIGHT_SPEED_/(1 + 2 * abs(steer)), STRAIGHT_SPEED_);
    //ROS_INFO("PathPlanner: Velocity = %f", vel);
  }
  return vel;
}

/**
 * @brief converts from the car frame to the occupancy grid frame
 * 
 * @param x the x coordinate of the point 
 * @param y the y coordinate of the point
 * @return int the index in the map frame of reference
 */
int PathPlanner::xyToMapIndex(double x, double y)
{
  // Convert [m] to [pixel] in car coordinate. (origin at bottom middle of image)
  double grid_x = round(x/resolution_);
  double grid_y = round(y/resolution_);

  // Convert [pixel] in car coordinate to [pixel] in image frame
  double map_x  = grid_x + round( map_W_/2 );
  double map_y  = -grid_y + map_H_;

  // Convert [pixel] in image frame to index in occupancy grid format
  int Map_index = (map_x-1)+(map_y-1)*map_W_;   //**map index starts from 0,0 from top left
  return Map_index;
}

/**
 * @brief returns the distance needed to stop when moving at a certain velocity 
 * 
 * @param vel1 the last commanded velocity
 * @return double the distance required to come to a complete stop
 */
double PathPlanner::StopDistFromVel(geometry_msgs::Twist velocity)
{
  return (pow(GetVelocityMagnitude(velocity), 2) * STOPPING_FACTOR_);
}
