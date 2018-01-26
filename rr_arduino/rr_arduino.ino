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


//!Serial defines
#define ROS_BAUD_RATE         115200
#define EC_BAUD_RATE          115200

//!Velocity encoder defines
#define ENCODER_FREQUENCY     20
#define ENCODER_PIN           47

Car RobotRacer;

int encoder_counts_to_meters = 28673; //!< MUST be a decimal number
Encoder Encoder(ENCODER_PIN, encoder_counts_to_meters, ENCODER_FREQUENCY);

//!PID tuning parameter
float throttle_PID_val[3] = {0, 0, 0};
float rr_velocity = 0.0f , goal_velocity = 0.0f, autonomous_throttle = 0.0f;

//!PID initialization
PID ThrottlePID(&rr_velocity, &autonomous_throttle, &goal_velocity,
                throttle_PID_val[0], throttle_PID_val[1], throttle_PID_val[2], DIRECT);

/**********************************************************/
//               ROS communication setup                  //
/**********************************************************/
ros::NodeHandle nh;

std_msgs::Float32 cmdVelocity;
std_msgs::Float32 cmdSteering;

std_msgs::Int8 state_msg;
std_msgs::Float32 actual_velocity_msg;
std_msgs::Float32 debug;

//!Prototypes for Callbacks
void cmdVelocityCallback(const std_msgs::Float32 & cmd_vel_msg);
void cmdSteeringCallback(const std_msgs::Float32 & cmd_str_msg);
void ThrottlePIDArrayCallback(const std_msgs::Float32MultiArray & array);

//!ROS publisher and subscriber commands
ros::Publisher state("/arduino/vehicle_state", &state_msg);
ros::Publisher encoder("/arduino/enc_vel", &actual_velocity_msg);
ros::Publisher debugger ("/arduino/debug", &debug);
ros::Subscriber <std_msgs::Float32> velocity_msg ("/PathPlanner/vel_level", cmdVelocityCallback);
ros::Subscriber <std_msgs::Float32> steering_msg ("/PathPlanner/steer_cmd", cmdSteeringCallback);
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
  nh.subscribe(velocity_msg);
  nh.subscribe(steering_msg);
  Encoder.setup();

  RobotRacer.setup();
  //!PID setup
  ThrottlePID.SetMode(AUTOMATIC); //!<turn the PID on
  ThrottlePID.SetOutputLimits(NEUTRAL, THROTTLE_MAX); //!<sets range for throttle PID contoller
  ThrottlePID.SetSampleTime(20); //!<PID algorithm evaluates every 50ms
  ThrottlePID.SetTunings(333, 0,0.0);
  pinMode(13,OUTPUT);
}

long previous_time = 0; //!<for update encoder speed loop

void loop() {
  RobotRacer.RC_read(); //!<get RC controller values
  nh.spinOnce();
  long current_time = millis();
  float time_diff = (current_time - previous_time)/1000.0;
  if(Encoder.updateSpeed(time_diff,rr_velocity))
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
      Serial.print("AUTO ");
      ThrottlePID.Compute();
      RobotRacer.SetThrottle((int)autonomous_throttle);
      //!maps angle from -30 to 30 deg to MIN_STEER_VAL to MAX_STEER_VAL
      RobotRacer.SetSteering((int)steeringAngle);
   
      break;
  }
  state_msg.data = RobotRacer.GetState();
  state.publish(&state_msg);
  
}
