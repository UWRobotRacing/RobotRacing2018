
//ROS publisher commands
ros::Publisher imu_pub(&imu_msg);
ros::Publisher magnetic_pub(&magnetic_msg);


//node initialization
ros::NodeHandle nh;
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);



void setup(void) 
{
  Serial.begin(IMU_BAUD_RATE);
  Serial.println("Sensor Test"); Serial.println("");
  nh.advertise(imu_pub);
  nh.advertise(magnetic_pub);
  

  
  //Initialise the sensor
  if(!bno.begin())
  {
    //Detecting BNOO55
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}
/*
 *@brief function reads and publishes two messages, each that obtain its data from IMU readings.   
 *@param none
 *@returns void
 */ 
void imu_readings()
{
  //Get a new sensor event
  sensors_event_t event; 
  bno.getEvent(&event);

  //Get current time
  ros::Time current_time=ros::Time::now();
  //assigning values for imu message
  imu_msg.orientation.x=event.orientation.x;
  imu_msg.orientation.y=event.orientation.y;
  imu_msg.orientation.z=event.orientation.z;

  imu_msg.linear_acceleration.x=acc.x;
  imu_msg.linear_acceleration.y=acc.y;
  imu_msg.linear_acceleration.z=acc.z;

  imu_msg.angular_velocity.x=gyro.x;
  imu_msg.angular_velocity.y=gyro.y;
  imu_msg.angular_velocity.z=gyro.z;

  imu_msg.header.stamp=current_time;
  imu_pub.publish(&imu_msg);
  //assigning values for magnetic field message
  magnetic_msg.header.stamp=current_time;
  magnetic_msg.magnetic_field.x=mag.x;
  magnetic_msg.magnetic_field.y=mag.y;
  magnetic_msg.magnetic_field.z=mag.z;
  magnetic_pub(&magnetic_msg);
  
}
void loop() 
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro= bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag= bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
/* Display the floating point data */
  Serial.print("XA: ");
  Serial.print(acc.x, 4);
  Serial.print(" YA: ");
  Serial.print(acc.y, 4);
  Serial.print(" ZA: ");
  Serial.print(acc.z, 4);
  Serial.println("\n");

  
/* Display the floating point data */
  Serial.print(" GX: ");
  Serial.print(gyro.x, 4);
  Serial.print(" GY: ");
  Serial.print(gyro.y, 4);
  Serial.print(" GZ: ");
  Serial.print(gyro.z, 4);
  Serial.println("\n");

  Serial.print(" MX: ");
  Serial.print(mag.x, 4);
  Serial.print(" MY: ");
  Serial.print(mag.y, 4);
  Serial.print(" MZ: ");
  Serial.print(mag.z, 4);
  Serial.println("\n");
  
  
  
  /* Display the floating point data */
  Serial.print(" X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.roll, 4);
  Serial.print("\n");
  
  
  delay(1000);
  //call function to publish imu_readings
  imu_readings();
}
