/*
 * @file Robot Racer Controller                                                       
 * @author Tom Meredith
 * @author Noah Abradjian 
 * @author Toni Ogunmade  
 * @author Abhi Srikantharajah 
 * @author Brian Kibazohi
 * @competition IARRC 2018
 * Last Updated: June 1, 2018                                                   
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
#include <PID_v1.h>


//Serial, velocity and battery monitoring defines respectively
#define ROS_BAUD_RATE         57600
#define EC_BAUD_RATE          115200


//I2C address for encoder counter 
#define SLAVE_ADDRESS 0x07

//Battery monitoring defines
#define BATTERY_FREQUENCY     2
#define BATTERY_PIN           A0
#define AVERAGING_SIZE        5

/**
 *@brief function Call
 *publishes actual velocity & average battery value
 *@param current_time 
 *@returns void
 */
void get_battery_state(long current_time);

Car robot_racer;

int encoder_counts_to_meters = 28673; // MUST be a decimal number
Encoder Encoder(ENCODER_PIN, encoder_counts_to_meters, ENCODER_FREQUENCY);

//PID tuning parameter
//double throttle_PID_val[3] = {0, 0, 0};
double rr_velocity = 0.0f , goal_velocity = 0.0f, autonomous_throttle = 1500.0f;

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

// Callback functions used by subscriber
void cmd_velocity_callback(const std_msgs::Float32 & cmd_vel_msg);
void cmd_steering_callback(const std_msgs::Float32 & cmd_str_msg);
void throttle_PID_array_callback(const std_msgs::Float32MultiArray & array);

// ROS publisher and subscriber commands

ros::Publisher state_pub("/arduino/vehicle_state", &state_msg);
ros::Publisher encoder_pub("/arduino/enc_vel", &actual_velocity_msg);
ros::Publisher debugger_pub ("/arduino/debug", &debug);
ros::Publisher velDebugger_pub ("/arduino/velDebug", &velDebug);
ros::Publisher battery_pub("/arduino/battery_state", &battery_percentage_msg);
ros::Subscriber <std_msgs::Float32> velocity_sub ("/PathPlanner/vel_level", cmd_velocity_callback);
ros::Subscriber <std_msgs::Float32> steering_sub ("/PathPlanner/steer_cmd", cmd_steering_callback);
//ros::Subscriber <std_msgs::Float32MultiArray> PID_msg ("/Throttle_PID_array", throttle_PID_array_callback);

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
  Serial2.begin(EC_BAUD_RATE);

/*
 *ROS Node Handler setup
 *@param specified topics
 */
  nh.getHardware()->setBaud(ROS_BAUD_RATE);
  nh.initNode();
  nh.advertise(state_pub);
  nh.advertise(encoder_pub);
  nh.advertise(debugger_pub);
  nh.advertise(velDebugger_pub);
  nh.advertise(battery_pub);
  nh.subscribe(velocity_sub);
  nh.subscribe(steering_sub);
  Encoder.setup();

  robot_racer.setup();
/**
 *PID setup
 *ThrottlePID.SetMode(AUTOMATIC); //<turn the PID on
 *ThrottlePID.SetOutputLimits(NEUTRAL, THROTTLE_MAX); //<sets range for throttle PID contoller
 *ThrottlePID.SetSampleTime(20); //<PID algorithm evaluates every 50ms
 *ThrottlePID.SetTunings(333, 0,0.0);
 */
  pinMode(13,OUTPUT);
}

/**
 *@brief we use RC values and car state to publish actual_velocity and state_msg
 *@param no input
 *@returns void
 */
long previous_time = 0; //<for update encoder speed loop
void loop() {
  robot_racer.RC_read(); //<get RC controller values
  nh.spinOnce();
  long current_time = millis();
  long double time_diff = (current_time - previous_time)/1000.0;
  if(Encoder.updateSpeed(time_diff,(float &) rr_velocity))
  {
    previous_time = millis();
    actual_velocity_msg.data = rr_velocity;
    encoder.publish(&actual_velocity_msg);
  }
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

  get_battery_state(current_time);
}
/*
*@brief publishes battery percentage at a specified BATTERY_FREQUENCY
*@param current time
*@returns void
*/
void get_battery_state(long current_time){
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
