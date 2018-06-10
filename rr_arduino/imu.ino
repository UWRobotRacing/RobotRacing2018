#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TEST_OUTPUT
#define IMU_BAUD_RATE       38400
#define IMU_SDA             A4
#define IMU_SCL             A5

//creating message objects
std_msgs::Float32 X;
std_msgs::Float32 Y;
std_msgs::Float32 Z;

//node initialization
ros::NodeHandle nh;

Adafruit_BNO055 bno = Adafruit_BNO055(55);


//X, Y ans Z variables represent accelerometer, magnetometer and gyroscope readings
void setup(void)
{
Serial3.begin(IMU_BAUD_RATE);
Serial.println("Orientation Sensor Test"); Serial.println("");
nh.advertise(X_pub);
nh.advertise(Y_pub);
nh.advertise(Z_pub);


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
*@brief function reads and prints the input from IMU
*@param none
*@returns void
*/
void imu_readings();
{
X_msg.data=event.orientation.x
Y_msg.data=event.orientation.y
Z_msg.data=event.orientation.z

X_pub.publish(&X_msg);
Y_pub.publish(&Y_msg);
Z_pub.publish(&Z_msg);
}
void loop(void)
{
/* Get a new sensor event */
sensors_event_t event;
bno.getEvent(&event);

/* Display the floating point data */
Serial.print("X: ");
Serial.print(event.orientation.x, 4);
Serial.print("\tY: ");
Serial.print(event.orientation.y, 4);
Serial.print("\tZ: ");
Serial.print(event.orientation.z, 4);
Serial.println("");

delay(1000);
//call function to publish imu_readings
imu_readings();
}


//ROS publisher commands
ros::Publisher X_pub("/arduino/X",&X_msg);
ros::Publisher Y_pub("/arduino/Y",&Y_msg);
ros::Publisher Z_pub("/arduino/Z",&Z_msg);
