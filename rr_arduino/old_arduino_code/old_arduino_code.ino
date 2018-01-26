/**********************************************************/
//               Robot Racer Controller                   //
//               Written By: Sirui Song                   //
//               Last Updated: Jun 30, 2014               //
/**********************************************************/
//#define TEST_OUTPUT

//Import Libraries
#include <Servo.h>       
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <SPI.h>

//servo
Servo throttleServo, steerServo,   brakeServo;
#define SERVO_THROTTLE_PIN    10
#define SERVO_STEER_PIN       11
#define SERVO_BRAKE_PIN       12
#define MANUAL_MAX            1575
#define MANUAL_REV_MAX        1375
#define THROTTLE_MAX          1675  // 1675 is 10 m/s, max. is 2000
#define THROTTLE_REV_MAX      1300  // Under 1300 is too fast, min. is 1000
#define MAX_RC_VAL            1738
#define MIN_RC_VAL            304
#define REST_RC_VAL           874
#define MAX_STEER_VAL         1732
#define MIN_STEER_VAL         306
#define REST_STEER_VAL        990
#define MAX_STEERING          2000
#define MIN_STEERING          1000
#define NEUTRAL               1500
#define STEER_NEUTRAL         1460
#define ROS_BAUD_RATE         115200 //115
#define EC_BAUD_RATE          115200 //115

unsigned int throttle_auto  = 1500;
unsigned int steering_auto  = 1500;
unsigned int throttle_rc    = 1500;
unsigned int steering_rc    = 1500;
unsigned int brake_rc       = 1500;
unsigned int throttle       = 1500;
unsigned int steering       = 1500;
unsigned int brake          = 1500;

unsigned int prev_steering = 1500;  

// Constants for RC mode
float x_reverse;
float x_forward;
float x_left;
float x_right;

bool enable = false;

//Define Analog Input Pin
#define BATT_VIN_1_PIN        A0
#define BATT_VIN_2_PIN        A1  
#define STR_ENCODER_PIN       A2

/*Velocity encoder*/
#define Hz                    40
#define PPR                   128          //512 for full Quad Decoding, 
#define PulsetoM              160.0      //48.2 was old value, value found with guess and check, combined with conversion from usecs to secs // MAKE SURE THIS IS A DECIMAL NOT AN INTEGER!!!!!!!!!!!!!!!

//Encoder pin assignment
int ss2 = 47; //


/**********************************************************/
//                   Helper Functions                     //
/**********************************************************/
double servo_scale(int value, int _min, int _max) {
  int delta = _max - _min;
  return (double) 180 *  (value-_min - delta/2)/delta;
};

/**********************************************************/
//                     RC Read In                         //
/**********************************************************/
int ppm[8];
void RC_read ()
{
  if (Serial2.available() > 0){
    int count00       = 0;
    int buffer_size   = 0;
    int incomingByte  = 0;
    while (count00 < 50){
      // throw away extra bytes
      if (Serial2.available() > 16) {
        incomingByte = Serial2.read();
        count00 = 0;
      } 
      else {
        count00++;
      }

      // wait for a full transmition
      if (Serial2.available() != buffer_size)
        count00 = 0;

      // make sure we are at the end of a transmition
      buffer_size = Serial2.available();
    }

    if (Serial2.available() == 16) {
      int ppm_data  = 0;
      int ppm_ch    = 0;
      for (unsigned int i=0; i<16; i++){
        incomingByte = Serial2.read();
        if (i >= 2) {
          if (i%2 == 0) {
            ppm_ch        = (incomingByte >> 3);
            incomingByte -= (ppm_ch << 3);
            ppm_data      = (incomingByte << 8);

            if (ppm_ch > 7) break;
          } 
          else {
            ppm_data     += incomingByte;
            ppm[ppm_ch]   = ppm_data;
          }
        }
      }
    }
  }

#ifdef TEST_OUTPUT
  Serial.print("RX Data: ");
  Serial.print(ppm[0], DEC); // throttle
  Serial.print(",");
  Serial.print(ppm[1], DEC); // aliron
  Serial.print(",");
  Serial.print(ppm[2], DEC); // elevator --> used as throttle
  Serial.print(",");
  Serial.print(ppm[3], DEC); // rudder --> used as steering
  Serial.print(",");
  Serial.print(ppm[4], DEC); // gear
  Serial.print(",");
  Serial.print(ppm[5], DEC); // RC/Auto
  Serial.print(",");
  Serial.print(ppm[6], DEC); // ch 7
  Serial.print("\n");
#endif

}
/**********************************************************/
//               ROS communication setup                  //
/**********************************************************/

ros::NodeHandle nh;

std_msgs::Float32 cmdVelocity;
std_msgs::Float32 cmdSteering;
std_msgs::Int8 state_msg;
std_msgs::Float32 encoder_msg;

ros::Publisher state("/arduino/vehicle_state", &state_msg);
ros::Publisher encoder("/arduino/enc_vel", &encoder_msg); 

void cmd_vel_cb(const std_msgs::Int32& cmd_vel_msg);
void cmd_str_cb(const std_msgs::Float32& cmd_str_msg);

ros::Subscriber <std_msgs::Int32> velocity_msg ("/controller/vel_cmd", cmd_vel_cb); 
ros::Subscriber <std_msgs::Float32> steering_msg ("/PathPlanner/steer_cmd", cmd_str_cb); 

int ROS_watchdog = 0;

void cmd_vel_cb(const std_msgs::Int32& cmd_vel_msg)
{
  ROS_watchdog = 0;
  // throttle_auto = round(cmd_vel_msg.data*20.7+1545);
  double vel_cmd = cmd_vel_msg.data;
  if (vel_cmd > 175)
    vel_cmd = 175;
  else if (vel_cmd < 0)
    vel_cmd = 0;
  throttle_auto = round(vel_cmd+1500);
  return;
}

void cmd_str_cb(const std_msgs::Float32& cmd_str_msg)
{
  ROS_watchdog = 0;
  double steeringAngle = min(cmd_str_msg.data, 30*PI/180); //30
  steeringAngle = max(steeringAngle, -30*PI/180);          //30
  
//  // Add Damping
//  if (steeringAngle >= 10*PI/180)
//  {
//    steeringAngle -= (steeringAngle/(30*PI/180))*(1*PI/180); //5
//  } else {
//    steeringAngle += (steeringAngle/(30*PI/180))*(1*PI/180); //5
//  }  
  
  steering_auto = round(-450*steeringAngle/(25*PI/180)+STEER_NEUTRAL);
  return;
}

//ESC Setup
void setup_ESC()
{
  long tmp_time = millis();

  while ( (millis() - tmp_time) <5000)
  {
    throttleServo.writeMicroseconds(1500);
  }
}

//Encoder Counter Functions
void setup_counter(int ss)
{
  digitalWrite(ss,HIGH);
  digitalWrite(ss,LOW);
  SPI.transfer(0b00100000); //Write to counter
  digitalWrite (ss,HIGH);

  digitalWrite(ss,HIGH);
  digitalWrite(ss,LOW);
  SPI.transfer(0b10001000); //10001xxx Write to MDR0
  SPI.transfer(0b00000011); //00000011
  digitalWrite (ss,HIGH);

  delay (10);

  digitalWrite(ss,HIGH);
  digitalWrite (ss,LOW); 
  SPI.transfer(0b10010000); //10010xxx Write to MDR1
  SPI.transfer(0b00000010); //00000010
  digitalWrite(ss,HIGH); 

}

int enc_pos_read(int ss)
{

  digitalWrite(ss,HIGH);
  digitalWrite(ss,LOW);
  SPI.transfer(0b01100000);
  byte response1 = SPI.transfer (0);
  byte response2 = SPI.transfer (0);
  digitalWrite(ss,HIGH);
  int temp = 0;
  temp = ((response1 << 8)|response2) ;
  return temp;

}

//reset counter only
void enc_pos_reset(int ss)
{
  digitalWrite(ss,HIGH);
  digitalWrite(ss,LOW);
  SPI.transfer(0b00100000);
  digitalWrite(ss,HIGH);

}
void setup()
{
  #ifdef TEST_OUTPUT
    Serial.begin(ROS_BAUD_RATE);
  #endif
  
  Serial2.begin(EC_BAUD_RATE);

  nh.getHardware()->setBaud(ROS_BAUD_RATE);
  nh.initNode();  
  nh.advertise(state);
  nh.advertise(encoder);
  nh.subscribe(velocity_msg);
  nh.subscribe(steering_msg);
  
  throttleServo.attach(SERVO_THROTTLE_PIN);
  steerServo.attach(SERVO_STEER_PIN);
  brakeServo.attach(SERVO_BRAKE_PIN);

  //Velocity Encoder Setup
  pinMode(ss2,OUTPUT);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  delay(1000);

  setup_counter(ss2); 
  setup_ESC();

  // Constants for RC mode control
  x_reverse = (MIN_RC_VAL - REST_RC_VAL)/(MANUAL_REV_MAX - NEUTRAL);
  x_forward = (MAX_RC_VAL - REST_RC_VAL)/(MANUAL_MAX - NEUTRAL);
  x_left = (MAX_STEER_VAL - REST_STEER_VAL)/(MAX_STEERING - STEER_NEUTRAL);
  x_right = (MIN_STEER_VAL - REST_STEER_VAL)/(MIN_STEERING - STEER_NEUTRAL);
}

long CurrentTime = 0;
long PrevTime = micros();
long timeDiff = CurrentTime - PrevTime;
float filterDegree = 0.70; // Amount of filtering, on a scale of 0 to 1 where 0 is no filtering and 1 is max filtering

void loop()
{  
  nh.spinOnce(); 
  RC_read ();

  /**********************************************************/
  //                      Encoder Read                      //
  /**********************************************************/
  // Use the other method --> look ahead 1000 ms or something
  CurrentTime = micros();
  timeDiff = CurrentTime - PrevTime;  
  if (timeDiff >= (1.0/Hz * 1E6)) 
  {    
    int ENC_M_CNT = enc_pos_read(ss2);
    
    // Low-pass filter for encoder readings
    float encData = ENC_M_CNT*PulsetoM/timeDiff;
    float filterEncVel = encData * (1 - filterDegree) + (filterEncVel * filterDegree);
    float roundedEncVel = round(filterEncVel * 100) / 100.0;
    encoder_msg.data = roundedEncVel;

    // Publishing encoder velocity
    encoder.publish (&encoder_msg);

    enc_pos_reset(ss2);

    PrevTime = CurrentTime;

    state.publish(&state_msg);
  } 

  /**********************************************************/
  //                      State Machine                     //
  /**********************************************************/
  boolean RC_on         = true;              //True for RC, false for Auto 
  boolean Estop_on      = true;              //True for RC, false for Auto

  if (ppm[4] < 700) 
  {
    Estop_on = true;
  }
  else
  {
    Estop_on = false;
  }

  if (Estop_on) {
#ifdef TEST_OUTPUT
    Serial.println("ESTOP MODE");
#endif 
    //Estop state:  
    state_msg.data = 0;
    //state.publish(&state_msg);   

    while(throttle >= 1500)
    {
      throttle -= 25;
      throttleServo.writeMicroseconds(throttle);
      delay(100);

    }//end while 
    steering = prev_steering;
    brake = 2000;

    //Reset Integrator 
  }
  else if (ppm[0] < 1500) {
  
    //RC state
    //Transfer PWM, with a max output limited
    state_msg.data = 1;

    // Equation was formed using previously defined formulas
    if(ppm[2] >= REST_RC_VAL)
    {
      throttle_rc = round((ppm[2] - REST_RC_VAL) / x_forward) + NEUTRAL;
    }
    else
    {
      throttle_rc = round((ppm[2] - REST_RC_VAL) / x_reverse) + NEUTRAL;
    }


/*******************
ppm[3] value:
max 1720, when the joy stick is pushed to the leftmost.
centered at 990 when recovered from the right;
centered at 1022 when recovered from left;
min 306, when the joy stick is pushed to the rightmost.
*********************/

    // Many of the numerical values were previously defined values
    int steering_read = ppm[3];

    // Give range for steering so that it stays neutral through fluctuations in RC values
    if(steering_read >= 960 && steering_read <= 1020)
    {  
      steering_rc = STEER_NEUTRAL;
      //steering_rc = round(-1*(steering_read-960)/1.308)+NEUTRAL;
    }
    else if(steering_read > 1020)
    {
      steering_rc = round(-1 * (steering_read - REST_STEER_VAL) / x_left) + STEER_NEUTRAL;
    }
    else
      steering_rc = round(-1 * (steering_read - REST_STEER_VAL) / x_right) + STEER_NEUTRAL;
    
    brake_rc = round((ppm[0] - 1022) / 1.432) + NEUTRAL;

    throttle = throttle_rc;
    steering = steering_rc;
    brake = brake_rc;
    
#ifdef TEST_OUTPUT
    Serial.println("RC MODE"); 
    Serial.print("Throttle PWM: ");  
    Serial.println(throttle); 
#endif 

  }
  else
  {
#ifdef TEST_OUTPUT
    Serial.println("AUTO MODE"); 
#endif 

    //Auto state
    //Output to motor: (May implement a min(input, from computer) for throttle
    state_msg.data = 2;
    //state.publish(&state_msg);
    
    throttle = throttle_auto;
    steering = steering_auto;
    
    if (steering > STEER_NEUTRAL)
    {
      steering += 5;
    }
    
    /*if (enable)
    {
        throttle = throttle_auto;
        steering = steering_auto;
    }
    else
    {
      throttle = NEUTRAL;
      steering = STEER_NEUTRAL;
    }*/
  }

  if (throttle > THROTTLE_MAX)
    throttle = THROTTLE_MAX;
  else if (throttle < THROTTLE_REV_MAX)
    throttle = THROTTLE_REV_MAX;
    
  if (steering > MAX_STEERING)
    steering = MAX_STEERING;
  else if (steering < MIN_STEERING)
    steering = MIN_STEERING; 
    
  // Steering test
  
//  double test_rad = 0.4014;
//  double steeringAngle = min(test_rad, 28*PI/180);
//  steeringAngle = max(steeringAngle, -28*PI/180);
//  steering = round(-450*steeringAngle/(25*PI/180)+1500);
//  

  steerServo.writeMicroseconds(steering);
  throttleServo.writeMicroseconds(throttle);
  brakeServo.writeMicroseconds(brake);
  delay(10);
  
  prev_steering = steering;
}
