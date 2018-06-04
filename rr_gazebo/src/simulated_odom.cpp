/**
 * @brief 
 * 
 * @file simulated_odom.cpp
 * @author Toni Ogunmade(oluwatoni)
 * @date 2018-05-04
 */

#include "simulated_odom.hpp"

/**
 * @brief Construct a new Simulated Odom:: Simulated Odom object
 * 
 */
SimulatedOdom::SimulatedOdom() {
  // the noise sd for the angular and linear velocities
  if(!nh_.getParam("simulated_odometry/angular_velocity_sd", angular_vel_sd_)){
    ROS_ERROR("angular velocity standard deviation was not specified");
    ros::shutdown();
  }
  if(!nh_.getParam("simulated_odometry/linear_velocity_x_sd", linear_vel_x_sd_)){
    ROS_ERROR("linear velocity x standard deviation was not specified");
    ros::shutdown();
  }
  if(!nh_.getParam("simulated_odometry/linear_velocity_y_sd", linear_vel_y_sd_)){
    ROS_ERROR("linear velocity y standard deviation was not specified");
    ros::shutdown();
  }
  if(!nh_.getParam("simulated_odometry/right_rear_wheel/diameter", wheel_diameter_)){
    ROS_ERROR("wheel diameter was not specified");
    ros::shutdown();
  }
  if(!nh_.getParam("simulated_odometry/wheel_to_wheel_dist", wheel_to_wheel_dist_)){
    ROS_ERROR("the wheel to wheel distance was not specified");
    ros::shutdown();
  }

  // initialize the odometry message
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  odom_.pose.pose.position.x = 0;
  odom_.pose.pose.position.y = 0;
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation.w = 1;
  odom_.pose.pose.orientation.x = 0;
  odom_.pose.pose.orientation.y = 0;
  odom_.pose.pose.orientation.z = 0;
  // Covariance for position is not important
  odom_.pose.covariance = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;
  // Covariance for orientation is not important
  odom_.twist.covariance = {linear_vel_x_sd_, 0, 0, 0, 0, 0,
                            0, linear_vel_y_sd_, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, angular_vel_sd_};
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

  // create the subscriber
  joint_sub_ = nh_.subscribe("/rr_vehicle/joint_states", 1, &SimulatedOdom::jointStateCb, this);

  then_ = ros::Time::now();
}

void SimulatedOdom::jointStateCb(const sensor_msgs::JointState::ConstPtr &msg) {
  double dt = msg->header.stamp.toSec() - then_.toSec();
  then_ = msg->header.stamp;
  // get the steering data
  double steering_angle = (msg->position[LEFT_STEERING] +
                           msg->position[RIGHT_STEERING]) / 2.0;
  // ROS_INFO("steering: %f radians", steering_angle);

  // get each wheels velocity
  double wheel_velocity = (msg->velocity[FRONT_LEFT_WHEEL] +
                           msg->velocity[FRONT_RIGHT_WHEEL]) / 2.0;
  // convert it to linear velocity
  wheel_velocity *= (wheel_diameter_ / 2.0);
  // ROS_INFO("wheel speed: %f m/s", wheel_velocity);
  double yaw, pitch, roll;
  tf::Quaternion q(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                   odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
  tf::Matrix3x3 ypr(q);
  ypr.getRPY(roll, pitch, yaw);
  //ROS_INFO("yaw: %f radians", yaw);

  // assuming the center of gravity is in the middle of the robot
  // Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design
  // http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
  double lf = wheel_to_wheel_dist_ * 0.5;
  double lr = wheel_to_wheel_dist_ - lf;
  double beta = atan2(lr * tan(steering_angle), lf + lr);

  double x_dot = wheel_velocity * cos(yaw + beta); 
  double y_dot = wheel_velocity * sin(yaw + beta); 
  double omega = (wheel_velocity / lr) * sin(beta);

  odom_.pose.pose.position.x += x_dot * dt;
  odom_.pose.pose.position.y += y_dot * dt;
  q = tf::createQuaternionFromYaw(yaw + (omega * dt));
  odom_.pose.pose.orientation.w = q.w();
  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.twist.twist.linear.x = x_dot;
  odom_.twist.twist.linear.y = y_dot;
  odom_.twist.twist.angular.z = omega; 

  // transform details
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = odom_.pose.pose.position.x;
  odom_trans.transform.translation.y = odom_.pose.pose.position.y;
  odom_trans.transform.rotation.w = q.w();
  odom_trans.transform.rotation.x = q.x();
  odom_trans.transform.rotation.y = q.y();
  odom_trans.transform.rotation.z = q.z();

  //publish the odometry and send the transform
  odom_pub_.publish(odom_);
  br_.sendTransform(odom_trans);
  //ROS_INFO("x: %f, y: %f, theta: %f", odom_.pose.pose.position.x, odom_.pose.pose.position.y);
}