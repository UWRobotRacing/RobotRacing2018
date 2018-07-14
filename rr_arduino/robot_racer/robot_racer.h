#ifndef ROBOT_RACER_H
#define ROBOT_RACER_H

#include "Servo.h"
#include "Arduino.h"
// #include <geometry_msgs/TransformStamped.h>


//! #define BRAKE
#define TEST_OUTPUT 1

//! servo defines
#define SERVO_THROTTLE_PIN    9
#define SERVO_STEER_PIN       11
#define SERVO_BRAKE_PIN       12
#define MANUAL_MAX            1575
#define MANUAL_REV_MAX        1375
#define THROTTLE_MAX          1675  //!< 1675 is 10 m/s, max. is 2000
#define THROTTLE_REV_MAX      1300  //!< Under 1300 is too fast, min. is 1000
#define MAX_RC_VAL            1738
#define MIN_RC_VAL            304
#define REST_RC_VAL           874

// RC Steering
#define STEER_NEUTRAL         1460


// Default Servo Parameters
#define STEERING_OFFSET       100
#define MAX_STEERING          2000 - STEERING_OFFSET
#define NEUTRAL               1500
#define MIN_STEERING          1000 + STEERING_OFFSET

// Actual Servo Parameters
#define REST_STEER_VAL        795 // 982
#define MAX_RC_STEER_VAL      1566
#define MIN_RC_STEER_VAL      281
#define DEADZONE_LOWER_BOUND  760 // 950
#define DEADZONE_UPPER_BOUND  830 // 1020


#define MAX_STEERING_ANGLE    0.5236
#define MIN_STEERING_ANGLE    (-0.5236)
#define THROTTLE_REDUCTION    25
#define BRAKE_DELAY           100  //!< Reducing this quickens the vehicle braking
#define RC_CHANNELS           8
#define DELAY                 250 // Time in milliseconds for estop to trigger when r/c disconnects

//! States for eStop, RC, and Auto
enum CarState
{
  ESTOP,
  RC,
  AUTO
};

//! Has all variables for the car
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

    float x_;
    float y_;
    float theta_;
    // geometry_msgs::TransformStamped odom_trans_;
    
    Servo ThrottleServo_, SteerServo_;
#ifdef BRAKE
    Servo BrakeServo_;//!< brakeServo not used?
#endif

  public:
    Car();
    enum CarState GetState();
    void setup();
    void SetThrottle(int newThrottle);
    void SetSteering(int newSteering);
    void SetState(CarState set);
    void Estop();
    void CheckController();
    void RC_read(int *incoming_bytes);
    void RCMode();
    void WriteToServos();
    long GetPreviousTime();
    void SetPreviousTime(long time);

    void GetOdomTrans();
    void GetOdomMsg();
    void RawToOdom(float vel, float str_angle);
};
#endif
