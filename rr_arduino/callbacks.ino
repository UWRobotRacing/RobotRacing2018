//ROS callbacks are automatically linked with through the Arduino IDE
void throttle_PID_array_callback(const std_msgs::Float32MultiArray & array) {
  // P,I,D respectively
  // for(int i = 0; i < 3; i++)
  // {
  //   //array[i] = throttle_PID_val[i];
  // }
  // ThrottlePID.SetTunings(throttle_PID_val[0], throttle_PID_val[1], throttle_PID_val[2]);
}

void cmd_velocity_callback(const std_msgs::Float32 & cmd_vel_msg)
{
  // goal_velocity = cmd_vel_msg.data;
  autonomous_throttle = cmd_vel_msg.data > 0 ? cmd_vel_msg.data* 21.5 + 1530: 1500;
  autonomous_throttle = constrain(autonomous_throttle, 1300, 1650);
  velDebug.data = autonomous_throttle;
  velDebugger.publish(&velDebug);
}

void cmd_steering_callback(const std_msgs::Float32 & cmd_str_msg)
{
  //limits steering input from -30 to 30 degrees
    steering_angle = ((((-1*cmd_str_msg.data)+ 0.5236)/(2* 0.5236))*(2000-1000))+ 960;
    steering_angle = constrain(steering_angle,980,2000);
    debug.data = steering_angle;
    debugger.publish(&debug);
}

