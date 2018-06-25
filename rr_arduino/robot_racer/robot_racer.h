#ifndef ROBOT_RACER_H
#define ROBOT_RACER_H

#include "Servo.h"
#include "Arduino.h"

// #define BRAKE
#define TEST_OUTPUT 1

// servo defines
const int SERVO_THROTTLE_PIN    10
const int SERVO_STEER_PIN       11
const int SERVO_BRAKE_PIN       12
const int MANUAL_MAX            1575
const int MANUAL_REV_MAX        1375
const int THROTTLE_MAX          1675  //< 1675 is 10 m/s, max. is 2000
const int THROTTLE_REV_MAX      1300  //< Under 1300 is too fast, min. is 1000
const int MAX_RC_VAL            1738
const int MIN_RC_VAL            304
const int REST_RC_VAL           874
const int MAX_RC_STEER_VAL      1726
const int MIN_RC_STEER_VAL      306
const int REST_STEER_VAL        982
const int MAX_STEERING          2000
const int MIN_STEERING          1000
const int NEUTRAL               1500
const int STEER_NEUTRAL         1460
const float MAX_STEERING_ANGLE  0.5236
const float MIN_STEERING_ANGLE  (-0.5236)
const int THROTTLE_REDUCTION    25
const int BRAKE_DELAY           100  //< Reducing this quickens the vehicle braking
const int RC_CHANNELS           8
const int DELAY                 250 // Time in milliseconds for estop to trigger when r/c disconnects

// States for eStop, RC, and Auto
enum CarState
{
  ESTOP,
  RC,
  AUTO
};

// Has all variables for the car
class Car
{
  private:
    int RC_signal_[RC_CHANNELS];
    CarState car_state_;    
    unsigned int steering_auto_;
    unsigned int throttle_rc_;
    unsigned int steering_rc_;
    unsigned int brake_rc_;
    unsigned int throttle_;
    unsigned int steering_;
    unsigned int brake_;
    unsigned int prev_steering_;
    float reverse_throttle_multiplier_;
    float forward_throttle_multiplier_;
    float left_steering_multiplier_;
    float right_steering_multiplier_;
    long previous_;
    
    Servo ThrottleServo_, SteerServo_;
#ifdef BRAKE
    Servo BrakeServo_;//< brakeServo not used?
#endif

  public:
    Car();
    enum CarState GetState();
    void setup();
    void SetThrottle(int newThrottle);
    void SetSteering(int newSteering);
    void SetState(CarState set);
    void Estop();
    void RC_read();
    void RCMode();
    void WriteToServos();
    long GetPreviousTime();
    void SetPreviousTime(long time);
};
#endif
