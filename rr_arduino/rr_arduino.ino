/*
 * @file Robot Racer Controller
 * @author Tom Meredith
 * @author Noah Abradjian
 * @author Toni Ogunmade
 * @author Abhi Srikantharajah
 * @author Brian Kibazohi
 * @competition IARRC 2018
 * Last Updated: June 24, 2018
 */

//#define TEST_OUTPUT

//Import Libraries
#include <SPI.h>
#include <Servo.h>
#include <robot_racer.h>
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>

//Serial, velocity and battery monitoring defines respectively
const float ROS_BAUD_RATE  =  57600;
const float RC_BAUD_RATE   =  115200;

//I2C address for encoder counter
const int SLAVE_ADDRESS = 07;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Battery monitoring defines
const int      BATTERY_FREQUENCY = 2;
const int      BATTERY_PIN       = A0;
const int      AVERAGING_SIZE    = 5;

/**
 *@brief function Call
 *publishes actual velocity & average battery value
 *@param current_time
 *@returns void
 */
void GetBatteryState(long current_time);

Car robot_racer;

//PID tuning parameter
//double throttle_PID_val[3] = {0, 0, 0};
double rr_velocity = 0.0f , goal_velocity = 0.0f, autonomous_throttle = 1500.0f;
// Initial position of the robot
double x = 0.0, y = 0.0, theta = 0.0;

//PID initialization
// PID ThrottlePID(&rr_velocity, &autonomous_throttle, &goal_velocity,
//                 throttle_PID_val[0], throttle_PID_val[1], throttle_PID_val[2], DIRECT);

//brief ROS communication setup begins

//node initialization

ros::NodeHandle nh;

// message objects created
std_msgs::Float32 cmdVelocity;
std_msgs::Float32 cmdSteering;
std_msgs::Int8 state_msg;
std_msgs::Float32 actual_velocity_msg;
std_msgs::Float32 debug;
std_msgs::Float32 velDebug;
std_msgs::Int8 battery_percentage_msg;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";

//creating IMU data message objects
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField magnetic_msg;

//Callback functions for subscriber
void CmdVelocityCallback(const std_msgs::Float32 &cmd_vel_msg);
void CmdSteeringCallback(const std_msgs::Float32 &cmd_str_msg);


// ROS publisher and subscriber commands

ros::Publisher state_pub("/arduino/vehicle_state", &state_msg);
ros::Publisher encoder_pub("/arduino/enc_vel", &actual_velocity_msg);
ros::Publisher debugger_pub ("/arduino/debug", &debug);
ros::Publisher velDebugger_pub ("/arduino/velDebug", &velDebug);
ros::Publisher battery_pub("/arduino/battery_state", &battery_percentage_msg);
ros::Publisher imu_pub("/arduino/imu_data", &imu_msg);
ros::Publisher magnetic_pub("/arduino/mag_data", &magnetic_msg);
ros::Subscriber <std_msgs::Float32> velocity_sub ("/PathPlanner/vel_level", CmdVelocityCallback);
ros::Subscriber <std_msgs::Float32> steering_sub ("/PathPlanner/steer_cmd", CmdSteeringCallback);

int steering_angle = 1500;
int ROS_watchdog = 0;

/**
 *@brief general setup for node handler & robot racer
 *sets the data rate for serial transmition
 *@param no input
 *@returns void
 */

void setup() {
#ifdef TEST_OUTPUT
  Serial.begin(ROS_BAUD_RATE);
#endif
  Serial2.begin(RC_BAUD_RATE);

/*
 *ROS Node Handler setup
 *@param specified topics
 */
  nh.initNode();
  nh.advertise(state_pub);
  nh.advertise(encoder_pub);
  nh.advertise(debugger_pub);
  nh.advertise(velDebugger_pub);
  nh.advertise(battery_pub);
  nh.advertise(imu_pub);
  nh.advertise(magnetic_pub);
  nh.subscribe(velocity_sub);
  nh.subscribe(steering_sub);

  robot_racer.setup();
/**
 *PID setup
 *ThrottlePID.SetMode(AUTOMATIC); //<turn the PID on
 *ThrottlePID.SetOutputLimits(NEUTRAL, THROTTLE_MAX); //<sets range for throttle PID contoller
 *ThrottlePID.SetSampleTime(20); //<PID algorithm evaluates every 50ms
 *ThrottlePID.SetTunings(333, 0,0.0);
 */
   pinMode(13,OUTPUT);
   digitalWrite (13, LOW);
   //if the imu isn't set up properly the LED should come on
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    digitalWrite(13, HIGH);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

/**
 *@brief we use RC values and car state to publish actual_velocity and state_msg
 *@param no input
 *@returns void
 */
long previous_time = 0; //<for update encoder speed loop
void loop()
{
  //robot_racer.RC_read(); //<get RC controller values
  nh.spinOnce();

  long current_time = millis();
  long double time_diff = (current_time - previous_time)/1000.0;
  /*if(Encoder.updateSpeed(time_diff,(float &) rr_velocity))
  {
    previous_time = millis();
    actual_velocity_msg.data = rr_velocity;
    encoder.publish(&actual_velocity_msg);`
  }*/

  switch (robot_racer.GetState()) {
//case for emergency stop
    case ESTOP:
      robot_racer.Estop();
      break;
//case for RC control

    case RC:
      robot_racer.RCMode();
      break;

//case for autonomous mode

    case AUTO:
      // ThrottlePID.Compute();
      robot_racer.SetThrottle((int)autonomous_throttle);
      //maps angle from -30 to 30 deg to MIN_STEER_VAL to MAX_STEER_VAL
      robot_racer.SetSteering((int)steering_angle);

      break;
  }
  state_msg.data = robot_racer.GetState();
  state_pub.publish(&state_msg);

  GetBatteryState(current_time);

  //call function to publish imu_readings
  ImuReadings();
}
/*
*@brief publishes battery percentage at a specified BATTERY_FREQUENCY
*@param current time
*@returns void
*/
void GetBatteryState(long current_time){
  static long prev_time = 0;
  if ((current_time - prev_time) * BATTERY_FREQUENCY >= 1000){
    long battery_value = analogRead(BATTERY_PIN);
    int battery_percentage = 100 * battery_value / 1024;

//Set up array for low pass averaging at first time

    static int percentages[AVERAGING_SIZE];
    if (prev_time == 0){
      for (int i = 0; i < AVERAGING_SIZE; i++){
        percentages[i] = battery_percentage;
      }
    }

// update array, calculate sum

    int sum = 0;
    for (int i = AVERAGING_SIZE - 1; i > 0; --i){
      percentages[i] = percentages[i - 1];
      sum += percentages[i];
    }
    percentages[0] = battery_percentage;
    sum += battery_percentage;

//publish the average value
    battery_percentage_msg.data = (int)(sum / AVERAGING_SIZE);
    battery_pub.publish(&battery_percentage_msg);

    prev_time = millis();
  }
}

void RawToOdom(double vel, double str_angle) {
  // Length of the car is 0.335 m
  double L = 0.335;
  long current_time = millis();
  long time_diff = current_time - previous_time;
  // If the robot is at the origin, calculate the position using steering angle and the velocity
  if (str_angle != 0.0 && x == 0.0 && y == 0.0) {
    // store str_angle in radians
    theta = str_angle * M_PI / 180.0;
    x = vel * cos(theta) * time_diff;
    y = vel * sin(theta) * time_diff;

  } else {
    // Kinematic equations used: https://nabinsharma.wordpress.com/2014/01/02/kinematics-of-a-robot-bicycle-model/
    // Calculate turning angle beta
    double d = vel * time_diff;
    double R = L / tan(str_angle);
    double beta = d / R;
    double xc = x - R * sin(theta);
    double yc = y + R * cos(theta);

    x = xc + R * sin(theta + beta);
    y = yc - R * cos(theta + beta);
    theta = (theta + beta) % (2 * M_PI);
  }

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  odom_trans.header.stamp = current_time;
  odom_trams.transform.translation.x = x;
  odom_trams.transform.translation.y = y;
  odom_trams.transform.translation.z = 0.0;
  odom_trams.transform.rotation = odom_quat;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
}
