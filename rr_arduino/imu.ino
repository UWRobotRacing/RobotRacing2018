

/*
 *@brief function reads and publishes two messages, each that obtain its data from BNO055.   
 *@param none
 *@returns void
 */ 
void ImuReadings()
{
  //Get a new sensor event
  sensors_event_t event; 
  bno.getEvent(&event);
  imu::Vector<3> acc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler= bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> mag  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  //Get current time
  ros::Time current_time=ros::Time::now();
  //assigning values for imu message

  imu_msg.linear_acceleration.x=acc.x();
  imu_msg.linear_acceleration.y=acc.y();
  imu_msg.linear_acceleration.z=acc.z();

  imu_msg.angular_velocity.x=gyro.x();
  imu_msg.angular_velocity.y=gyro.y();
  imu_msg.angular_velocity.z=gyro.z();

  imu_msg.orientation.x=euler.x();
  imu_msg.orientation.y=euler.y();
  imu_msg.orientation.z=euler.z();

  imu_msg.header.stamp=current_time;
  imu_pub.publish(&imu_msg);
  //assigning values for magnetic field message
  magnetic_msg.header.stamp=current_time;
  magnetic_msg.magnetic_field.x=mag.x();
  magnetic_msg.magnetic_field.y=mag.y();
  magnetic_msg.magnetic_field.z=mag.z();
  magnetic_pub(&magnetic_msg);
  
}
