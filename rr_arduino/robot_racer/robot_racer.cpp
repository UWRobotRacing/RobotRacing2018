/*
 * @file robot_racer
 * @competition IARRC 2018
 */

#include "robot_racer.h"
#include <nav_msgs/Odometry.h>
#include <math.h>

/*
 * @brief Constructs Car and assigns values to attributes
 * values can be found in robot_racer.h library
 */
Car::Car()
{
  car_state_      = RC;
  throttle_rc_    = NEUTRAL;
  steering_rc_    = NEUTRAL;
  brake_rc_       = NEUTRAL;
  throttle_       = NEUTRAL;
  steering_       = NEUTRAL;
  brake_          = NEUTRAL;
  prev_steering_  = NEUTRAL;
  reverse_throttle_multiplier_ = (MIN_RC_VAL - REST_RC_VAL) / (float)(MANUAL_REV_MAX - NEUTRAL);
  forward_throttle_multiplier_ = (MAX_RC_VAL - REST_RC_VAL) / (float)(MANUAL_MAX - NEUTRAL);
  left_steering_multiplier_ = (MAX_RC_STEER_VAL - REST_STEER_VAL) / (float)(MAX_STEERING - STEER_NEUTRAL);
  right_steering_multiplier_ = (MIN_RC_STEER_VAL - REST_STEER_VAL) / (float)(MIN_STEERING - STEER_NEUTRAL);
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
}
/*
 *@brief assigns pin values
 *@param no input
 *values can be found in robot_racer library
 *BRAKE currently has not been defined, when defined, a pin is assigned
 *@returns void
 */
void Car::setup()
{
  ThrottleServo_.attach(SERVO_THROTTLE_PIN);
  SteerServo_.attach(SERVO_STEER_PIN);

#ifdef BRAKE
  BrakeServo_.attach(SERVO_BRAKE_PIN);
#endif
}

/*
 *@brief returns the state of the car (EStop, RC, Auto)
 */
CarState Car::GetState()
{
  return car_state_;
}

/*
 *@brief next 2 methods set a new throttle and sterring respectively in the same fashion
 *new values are stored
 */
void Car::SetThrottle(int new_throttle)
{
  throttle_ = new_throttle;
  ThrottleServo_.writeMicroseconds(throttle_);
  return;
}

// sets a new steering value
void Car::SetSteering(int new_steering)
{
  steering_ = new_steering;
  SteerServo_.writeMicroseconds(steering_);
  return;
}

// sets new state for the car
void Car::SetState(CarState set)
{
  car_state_ = set;
  return;
}

/*
 *@brief Method for Estop car state
 *throttle is reduced to neutral, thats how the car gradually stops.
 *steering is re_adjusted to neutral
 *@returns void
 */
void Car::Estop()
{
#ifdef TEST_OUTPUT
	Serial.println("EStop");
#endif
  while (throttle_ >= NEUTRAL)
  {
    throttle_ -= THROTTLE_REDUCTION;
    ThrottleServo_.writeMicroseconds(throttle_);
    delay(BRAKE_DELAY);

  }
  //re-adjusts steering to neutral
  steering_= prev_steering_;
#ifdef BRAKE
  brake_ = 2000; // may not be used (for brake servo only)
#endif
  // TODO Reset Integrator
}

// For RC reading
void Car::RC_read()
{
// confirms and starts connection with RC
  if (Serial2.available() > 0) {
    int buffer_size   = 0;
    byte incoming_byte  = 0;

    // TODO understand why 50 and what this does
    for (byte i = 0; i < 50; i) {
      // throw away extra bytes, reads bytes from 17-49
      if (Serial2.available() > 16) {
        incoming_byte = Serial2.read();
        i = 0;
      }
      else {
        i++;
      }

      // wait for a full transmition
      if (Serial2.available() != buffer_size)
        i = 0;

      // make sure we are at the end of a transmition
      buffer_size = Serial2.available();
    }

    if (Serial2.available() == 16) {
      int RC_signal_data  = 0;
      int RC_signal_ch    = 0;
      for (unsigned int i = 0; i < 16; i++) {
        incoming_byte = Serial2.read();
        if (i >= 2) {
          if (i % 2 == 0) {
            RC_signal_ch        = (incoming_byte >> 3);
            incoming_byte -= (RC_signal_ch << 3);
            //RC_signal_data      = (incoming_byte << 8);
            RC_signal_data = (incoming_byte<< 8);//changed from above

            if (RC_signal_ch > 7) break;
            }
            else {
              RC_signal_data     += incoming_byte;
              RC_signal_[RC_signal_ch]   = RC_signal_data;
              SetPreviousTime(millis());
            }
          }
        }
      }
    }

/**
 *@brief relationship btwn the RC_signals and car state
 * the RC signal readings control the car state
 */
  if (RC_signal_[4] < 700)
  {
    car_state_ = ESTOP;
  }
  else if (RC_signal_[5] < 650) {
   car_state_ = RC;
  }
  else {
   car_state_ = AUTO;
  }
  /*
   *@brief If no messages are received, switch to EStop
   */
  // Serial.println(millis()-GetPreviousTime());
  if ((millis() - GetPreviousTime()) > DELAY){
  	car_state_ = ESTOP;
  }
  #ifdef TEST_OUTPUT
  Serial.print("RX Data: ");
  Serial.print(RC_signal_[0], DEC); // throttle
  Serial.print(",");
  Serial.print(RC_signal_[1], DEC); // aliron
  Serial.print(",");
  Serial.print(RC_signal_[2], DEC); // elevator --> used as throttle
  Serial.print(",");
  Serial.print(RC_signal_[3], DEC); // rudder --> used as steering
  Serial.print(",");
  Serial.print(RC_signal_[4], DEC); // gear
  Serial.print(",");
  Serial.print(RC_signal_[5], DEC); // RC/Auto
  Serial.print(",");
  Serial.print(RC_signal_[6], DEC); // ch 7
  Serial.print("\n");
#endif

}

/**
 * For RC MODE
*/
void Car::RCMode()
{
// REMOTE CONTROL ON
// Transfer PWM, with a max output limited

/*
 * RC_signal[2] value: forward/reverse throttle joystick
 * max 1738, when the joy stick is pushed to the topmost.
 * centered at 874;
 * min 304, when the joy stick is pushed to the bottommost.
 * forward and reverse ThrottleMultipliers deal with conversion to throttle
*/

/*
 *@noteEquation was formed using previously defined formulas
 */
  if (RC_signal_[2] >= REST_RC_VAL)
  {
    throttle_rc_ = round((RC_signal_[2] - REST_RC_VAL) / forward_throttle_multiplier_) + NEUTRAL;
  }
  else
  {
    throttle_rc_ = round((RC_signal_[2] - REST_RC_VAL) / reverse_throttle_multiplier_) + NEUTRAL;
  }


/*
 *@brief
 *RC_signal[3] value:
 *max 1720, when the joy stick is pushed to the leftmost.
 *centered at 990 when recovered from the right;
 *centered at 1022 when recovered from left;
 *min 306, when the joy stick is pushed to the rightmost.
*/

// Many of the numerical values were previously defined values
  int steering_read = RC_signal_[3];  //<left-right joystick

// Gives range for steering so that it stays neutral through fluctuations in RC values
  if (steering_read >= 950 && steering_read <= 1020)
  {
    steering_rc_ = STEER_NEUTRAL;
    // steering_rc_ = round(-1*(steering_read-960)/1.308)+NEUTRAL;
  }

  // for a steering value over the range given previously
  else if (steering_read > 1020)
  {
    steering_rc_ = round(-1 * (steering_read - REST_STEER_VAL) / left_steering_multiplier_) + STEER_NEUTRAL;
  }
  else
    steering_rc_ = round(-1 * (steering_read - REST_STEER_VAL) / right_steering_multiplier_) + STEER_NEUTRAL;
#ifdef BRAKE
  brake_rc_ = round((RC_signal_[0] - 1022) / 1.432) + NEUTRAL;
  brake_ = brake_rc_;
#endif
  throttle_ = throttle_rc_;
  steering_ = steering_rc_;

  WriteToServos();
}

/*
 *@brief writes to the throttle and steering servos
 */
void Car::WriteToServos()
{
  SteerServo_.writeMicroseconds(steering_);
  ThrottleServo_.writeMicroseconds(throttle_);
#ifdef BRAKE
  BrakeServo_.writeMicroseconds(brake_);
#endif
  //delay(10); //<CONSIDER OTHER DELAYS?(will check)
  //prev_steering_ = steering_;
}

/*
 *@brief Setting current time
 *@param long time
 * return void
*/
void Car::SetPreviousTime(long time){
	previous_ = time;
}

long Car::GetPreviousTime(){
	return previous_;
}

void Car::GetOdomTrans() {
  return odom_trans_;
}

void Car::GetOdomMsg() {
  return odom_;
}

void Car::RawToOdom(float vel, float str_angle) {
  // Length of the car is 0.335 m
  double L = 0.335;
  long current_time = millis();
  long time_diff = current_time - GetPreviousTime();
  // If the robot is at the origin, calculate the position using steering angle and the velocity
  if (str_angle != 0.0 && x_ == 0.0 && y_ == 0.0) {
    // store str_angle in radians
    theta_ = str_angle * M_PI / 180.0;
    x_ = vel * cos(theta_) * time_diff;
    y_ = vel * sin(theta_) * time_diff;

  } else {
    // Kinematic equations used: https://nabinsharma.wordpress.com/2014/01/02/kinematics-of-a-robot-bicycle-model/
    // Calculate turning angle beta
    double d = vel * time_diff;
    double R = L / tan(str_angle);
    double beta = d / R;
    double xc = x_ - R * sin(theta_);
    double yc = y_ + R * cos(theta_);

    x_ = xc + R * sin(theta_ + beta);
    y_ = yc - R * cos(theta_ + beta);
    theta_ = fmod((theta_ + beta),(2 * M_PI));
  }

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  odom_trans_.header.stamp = current_time;
  odom_trams_.transform.translation.x = x;
  odom_trams_.transform.translation.y = y;
  odom_trams_.transform.translation.z = 0.0;
  odom_trams_.transform.rotation = odom_quat;

  odom_.pose.pose.position.x = x;
  odom_.pose.pose.position.y = y;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation = odom_quat;
}
