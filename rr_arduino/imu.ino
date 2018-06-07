#define TEST_OUTPUT
#define IMU_BAUD_RATE       38400
#define IMU_SDA             A4
#define IMU_SCL             A5

//creating message objects
std_msgs::Float32 SDA;
std_msgs::Float32 SCL;


//node initialization
ros::NodeHandle nh;


void setup() {
  
  Serial3.begin(IMU_BAUD_RATE);
  nh.advertise(SDA_pub);
  nh.advertise(SCL_pub);
}
/*  
 *@brief function reads and prints the input from IMU   
 *@param none
 *@returns void
 */
void imu_readings() {

    SDA_reading=analogRead(IMU_SDA);
    SCL_reading=analogRead(IMU_SCL);
    return;

//prints data
    delay(10000)  
    Serial.println(SDA_reading,DEC);
    Serial.println(SCL_reading,DEC);
} 
//the function loops reading the IMU data
void loop() {
  void imu_readings();
  //assigning data to msgs
  SDA_msg.data=SDA_reading;
  SCL_msg.data=SCL_reading;
  delay(10000);
  SDA_pub.publish(&SDA_msg);
  SCL_pub.publish(&SCL_msg);

}




//ROS publisher commands
ros::Publisher SDA_pub("/arduino/SDA",&SDA_msg);
ros::Publisher SCL_pub("/arduino/SCL",&SCL_msg);

