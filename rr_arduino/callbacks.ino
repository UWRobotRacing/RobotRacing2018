//ROS callbacks are automatically linked with through the Arduino IDE


void cmdCallback(const geometry_msgs::Twist & cmd_msg)
{
  //use the bicycle model assumption to interpret the cmd cmd_msg
  //convert the velocities to a steering and throttle message
  float speed = sqrt(pow(cmd_msg.linear.x, 2) + pow(cmd_msg.linear.y, 2));

  if (cmd_msg.linear.x < 0 and cmd_msg.linear.y < 0)
  {
    speed *= -1;
  }
  float beta = 0;
  if (speed > 0.0001)
  {
    beta = asin(cmd_msg.angular.z / (speed /(WHEEL_TO_WHEEL_DIST / 2)));
  }
  steering_angle_raw = atan((tan(beta) * (WHEEL_TO_WHEEL_DIST)) / (WHEEL_TO_WHEEL_DIST / 2));

  autonomous_throttle = speed > 0 ? speed* 21.5 + 1530: 1500;
  autonomous_throttle = constrain(autonomous_throttle, 1300, 1650);
  velDebug.data = autonomous_throttle;

  //!limits steering input from -30 to 30 degrees
  steering_angle_signal = ((((-1*(steering_angle_raw))+ 0.5236)/(2* 0.5236))*(2000-1000))+ 960;
  steering_angle_signal = constrain(steering_angle_raw,980,2000);
}

