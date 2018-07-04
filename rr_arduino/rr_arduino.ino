/*********************************************************************************/
//  Robot Racer Controller                                                       //
//  Written By: Tom Meredith, Noah Abradjian, Toni Ogunmade, Abhi Srikantharajah //
//  Last Updated: Feb 26, 2016                                                   //
/*********************************************************************************/

//#define TEST_OUTPUT

//!Import Libraries
#include <SPI.h>
#include <grayhill_encoder_LS7266R.h>
#include <Servo.h>
#include <robot_racer.h>
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <PID_v1.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

//!Serial defines
#define ROS_BAUD_RATE         57600
#define EC_BAUD_RATE          115200

//!Velocity encoder defines
#define ENCODER_FREQUENCY     20
#define ENCODER_PIN           47

//Battery monitoring defines
#define BATTERY_FREQUENCY     2
#define BATTERY_PIN           A0
#define AVERAGING_SIZE        5

void get_battery_state(long current_time);

Car RobotRacer;

int encoder_counts_to_meters = 28673; //!< MUST be a decimal number
Encoder Encoder(ENCODER_PIN, encoder_counts_to_meters, ENCODER_FREQUENCY);

//!PID tuning parameter
// double throttle_PID_val[3] = {0, 0, 0};
double rr_velocity = 0.0f , goal_velocity = 0.0f, autonomous_throttle = 1500.0f;
// Initial position of the robot
double x = 0.0, y = 0.0, theta = 0;

//!PID initialization
// PID ThrottlePID(&rr_velocity, &autonomous_throttle, &goal_velocity,
//                 throttle_PID_val[0], throttle_PID_val[1], throttle_PID_val[2], DIRECT);

/**********************************************************/
//               ROS communication setup                  //
/**********************************************************/
ros::NodeHandle nh;

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

//!Prototypes for Callbacks
void cmdVelocityCallback(const std_msgs::Float32 & cmd_vel_msg);
void cmdSteeringCallback(const std_msgs::Float32 & cmd_str_msg);
void ThrottlePIDArrayCallback(const std_msgs::Float32MultiArray & array);

//!ROS publisher and subscriber commands
ros::Publisher state("/arduino/vehicle_state", &state_msg);
ros::Publisher encoder("/arduino/enc_vel", &actual_velocity_msg);
ros::Publisher debugger ("/arduino/debug", &debug);
ros::Publisher velDebugger ("/arduino/velDebug", &velDebug);
ros::Publisher battery("/arduino/battery_state", &battery_percentage_msg);
ros::Subscriber <std_msgs::Float32> velocity_msg ("/rr_vehicle/velocity_cmd", cmdVelocityCallback);
ros::Subscriber <std_msgs::Float32> steering_msg ("/rr_vehicle/steering_cmd", cmdSteeringCallback);
ros::Subscriber <std_msgs::Float32MultiArray> PID_msg ("/Throttle_PID_array", ThrottlePIDArrayCallback);

int steeringAngle = 1500;
int ROS_watchdog = 0;

void setup() {
#ifdef TEST_OUTPUT
  Serial.begin(ROS_BAUD_RATE);
#endif
  Serial2.begin(EC_BAUD_RATE);

  //!ROS Node Handler setup
  nh.getHardware()->setBaud(ROS_BAUD_RATE);
  nh.initNode();
  nh.advertise(state);
  nh.advertise(encoder);
  nh.advertise(debugger);
  nh.advertise(velDebugger);
  nh.advertise(battery);
  nh.subscribe(velocity_msg);
  nh.subscribe(steering_msg);
  Encoder.setup();

  RobotRacer.setup();
  //!PID setup
  // ThrottlePID.SetMode(AUTOMATIC); //!<turn the PID on
  // ThrottlePID.SetOutputLimits(NEUTRAL, THROTTLE_MAX); //!<sets range for throttle PID contoller
  // ThrottlePID.SetSampleTime(20); //!<PID algorithm evaluates every 50ms
  // ThrottlePID.SetTunings(333, 0,0.0);
  pinMode(13,OUTPUT);
}

long previous_time = 0; //!<for update encoder speed loop

void loop() {
  RobotRacer.RC_read(); //!<get RC controller values
  nh.spinOnce();
  long current_time = millis();
  long double time_diff = (current_time - previous_time)/1000.0;
  if(Encoder.updateSpeed(time_diff,(float &) rr_velocity))
  {
    previous_time = millis();
    actual_velocity_msg.data = rr_velocity;
    encoder.publish(&actual_velocity_msg);
  }
  switch (RobotRacer.GetState()) {
    case ESTOP: //!<case for emergency stop
      RobotRacer.Estop();
      break;

    case RC: //!<case for RC control
      RobotRacer.RCMode();
      break;

    case AUTO: //!<case for autonomous mode
      // ThrottlePID.Compute();
      RobotRacer.SetThrottle((int)autonomous_throttle);
      //!maps angle from -30 to 30 deg to MIN_STEER_VAL to MAX_STEER_VAL
      RobotRacer.SetSteering((int)steeringAngle);

      break;
  }
  state_msg.data = RobotRacer.GetState();
  state.publish(&state_msg);

  get_battery_state(current_time);
}

void get_battery_state(long current_time){
  static long prev_time = 0;
  //publish the battery percentage at specified frequency
  if ((current_time - prev_time) * BATTERY_FREQUENCY >= 1000){
    //Get and calculate battery percentage
    long battery_value = analogRead(BATTERY_PIN);
    int battery_percentage = 100 * battery_value / 1024;

    //Set up array for low pass averaging at first time
    static int percentages[AVERAGING_SIZE];
    if (prev_time == 0){
      for (int i = 0; i < AVERAGING_SIZE; i++){
        percentages[i] = battery_percentage;
      }
    }

    //update array, calculate sum
    int sum = 0;
    for (int i = AVERAGING_SIZE - 1; i > 0; --i){
      percentages[i] = percentages[i - 1];
      sum += percentages[i];
    }
    percentages[0] = battery_percentage;
    sum += battery_percentage;

    //publish the average value
    battery_percentage_msg.data = (int)(sum / AVERAGING_SIZE);
    battery.publish(&battery_percentage_msg);

    prev_time = millis();
  }
}

void RawToOdom(double vel, double str_angle) {
  // FIX ME: Get length of the car
  double L = 0;
  long current_time = millis();
  long time_diff = current_time - prev_time;
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
