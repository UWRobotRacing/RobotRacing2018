//!ROS callbacks are automatically linked with through the Arduino IDE
void ThrottlePIDArrayCallback(const std_msgs::Float32MultiArray & array) {
  //!P,I,D respectively
  // for(int i = 0; i < 3; i++)
  // {
  //   //array[i] = throttle_PID_val[i];
  // }
  // ThrottlePID.SetTunings(throttle_PID_val[0], throttle_PID_val[1], throttle_PID_val[2]);
}

void cmdVelocityCallback(const std_msgs::Float32 & cmd_vel_msg)
{
  // goal_velocity = cmd_vel_msg.data;
  autonomous_throttle = cmd_vel_msg.data > 0 ? cmd_vel_msg.data* 21.5 + 1530: 1500;
  autonomous_throttle = constrain(autonomous_throttle, 1300, 1650);
  velDebug.data = autonomous_throttle;
  velDebugger.publish(&velDebug);
}

void cmdSteeringCallback(const std_msgs::Float32 & cmd_str_msg)
{
  //!limits steering input from -30 to 30 degrees
    steeringAngle = ((((-1*cmd_str_msg.data)+ 0.5236)/(2* 0.5236))*(2000-1000))+ 960;
    steeringAngle = constrain(steeringAngle,980,2000);
    debug.data = steeringAngle;
    debugger.publish(&debug);
}

