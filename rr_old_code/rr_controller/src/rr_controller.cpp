/************************************************************************************
  Robot Racer Drag Race Controller
  Sirui Song
  Date:11/07/14

  This function uses PID to control the robot to go straight. Vision/camera is required
  Vision provides lateral and heading error in Pose2D msgs. x, and theta are the respective
  errors. 

      ** If enable is off, then all error resets. Velocity and steering are published as 0, 

************************************************************************************/
//#define TESTOUTPUT

#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <vector>
#include <cmath>
#include <iostream>

#define PI 3.14159

bool DEBUG;
bool enable;

//Controller Gains
double Kp_lat;
double Kd_lat;
double Ki_lat;
double lookahead;
double OFFSET_DISTANCE;
double OFFSET_HEADING;

double Kp_long;
double Kd_long;
double Ki_long;
double Kf_long;


//Global variables: 
double vel_error;
double de_vel_error;
double in_vel_error;
double vel_desired;

double heading_error;
double dheading_error;
double iheading_error;
double ANGLE_REJECT; 
double LAT_REJECT;

double cam_errorheading= 0;
double cam_lat_error= 0;

//Timing variable
double dt_camera = 0.01;
double dt_encoder = 0.01;

std_msgs::Float32 cmd_steering_msg;
std_msgs::Float32 cmd_throttle_msg;

ros::Subscriber enable_sub;
ros::Subscriber camera_sub;
ros::Subscriber velocity_sub;
ros::Publisher steering_publisher;
ros::Publisher velocity_publisher;

using namespace std;

void getParam ()
{
    ros::NodeHandle nh;
    nh.param<bool>("PIDController/DEBUG", DEBUG, false);
    nh.param<bool>("PIDController/Enabled", enable, false);

    nh.param<double>("PIDController/Kp_lat", Kp_lat, 1);
    nh.param<double>("PIDController/Kd_lat", Kd_lat, 0.0);
    nh.param<double>("PIDController/Ki_lat", Ki_lat, 0);
    nh.param<double>("PIDController/look_ahead", lookahead, 0.2);
    nh.param<double>("PIDController/OFFSET_DISTANCE", OFFSET_DISTANCE, 0);
    nh.param<double>("PIDController/OFFSET_HEADING", OFFSET_HEADING, 1);
    nh.param<double>("PIDController/ANGLE_REJECT", ANGLE_REJECT, 0.8);
    nh.param<double>("PIDController/LAT_REJECT", LAT_REJECT, 0.05);


    nh.param<double>("PIDController/Kp_long", Kp_long, 1);
    nh.param<double>("PIDController/Kd_long", Kd_long, 0.0);
    nh.param<double>("PIDController/Ki_long", Ki_long, 0);
    nh.param<double>("PIDController/Kf_long", Kf_long, 1);
    nh.param<double>("PIDController/Vel_Desired", vel_desired, 2);
}

//Calculates how much time has passed between msgs. only three timers right now. 
double time_elapsed(int ID){

    static ros::Time prev_t[3]= {ros::Time::now(), ros::Time::now(), ros::Time::now()}; 
    if (ros::ok()){

        double timediff = (ros::Time::now() - prev_t[ID]).toNSec()* 1E-3;
        prev_t[ID] = ros::Time::now();
        return timediff*1E-6;
    }
    return -1;
}

float PID (double error, double derror, double ierror, double desired, 
            double Kp,    double Kd,     double Ki,     double Kf) {

    if (isnan(error) || isnan(derror) || isnan(ierror) || isnan(desired)) {
        ROS_INFO("NaN fault detected in PID control loop");
        error = 0;
        derror = 0;
        ierror = 0;
        desired = 0;
    }

    return (float)Kp*error + Kd*derror + Ki*ierror + Kf * desired;
}

void long_error_calculation (double vel_measured, double dt){

    //To avoid integrator wind up, the enable pin should be tied to the Estop status 
    if (enable && dt > 0)
    {
        //calculate error
        double tmp_vel_error = vel_desired - vel_measured;
        de_vel_error = (tmp_vel_error-vel_error)/dt;
        in_vel_error = in_vel_error + tmp_vel_error;
        vel_error = tmp_vel_error;
       
        //call PID
        cmd_throttle_msg.data = PID(vel_error, de_vel_error, in_vel_error, vel_desired,
                                    Kp_long, Kd_long, Ki_long, Kf_long);
    }
    else {
        de_vel_error = 0;
        in_vel_error = 0;
        vel_error = 0;
        cmd_throttle_msg.data = 0;
    }
}

void lateral_error_calculation (double dt){
    
    double yawdesired= atan(cam_lat_error/lookahead);
    double yawerror = cam_errorheading - yawdesired;

    if (DEBUG) {
        std::cout << "yawdesired = " << yawdesired << std::endl; 
        std::cout << "lateral error = " << cam_lat_error << std::endl;
        std::cout << "errorHeading = " << cam_errorheading << std::endl;
    }

    //wrapping error
    if (yawerror > PI)
    {
        yawerror = 2*PI - yawerror;
    }
    else if (yawerror < -PI)
    {
        yawerror = 2*PI + yawerror;
    }  

    if (DEBUG) {
        std::cout << "yawError after wrap = " << yawerror << std::endl;
    }

    //To avoid integrator wind up, listen to enable, when disabled, integrator is off.  
    if (enable && dt > 0) {

        dheading_error = (yawerror-heading_error)/dt;
        iheading_error = in_vel_error + yawerror;
        heading_error = yawerror;

        cmd_steering_msg.data = PID(heading_error, dheading_error, iheading_error, 0,
                                    Kp_lat, Kd_lat, Ki_lat, 0);
    }
    else {        

        dheading_error = 0;
        iheading_error = 0;
        heading_error = 0;

        cmd_steering_msg.data = 0;
    }
}

void velocity_encoder_callback(const geometry_msgs::Point32 vel_e_msg){

    //This function should be changed to be receiving msg from estimators once fixed
    dt_encoder = time_elapsed(0);    
    long_error_calculation(vel_e_msg.y, dt_encoder);
    
    velocity_publisher.publish(cmd_throttle_msg);
}

void camera_callback(const geometry_msgs::Pose2D error_msg){
    //if (abs(error_msg.theta - cam_errorheading) < ANGLE_REJECT && 
    //    abs(cam_lat_error-error_msg.x * 0.001) < LAT_REJECT) {
        std::cout << "I'm here" << std::endl;    
        cam_errorheading = error_msg.theta-OFFSET_HEADING;
        cam_lat_error = -(error_msg.x * 0.001) + OFFSET_DISTANCE;
        dt_camera = time_elapsed(1);    
        lateral_error_calculation(dt_camera);

        //Publish the msgs. 
        steering_publisher.publish(cmd_steering_msg);
    //}
}

void enable_callback(const std_msgs::Bool& enable_msg) {
    enable = enable_msg.data;
}

int main(int argc, char **argv)
{
    //INITIALIZATION
    ros::init(argc,argv,"Robot_Racer_PID_Drag_Race");
    ros::NodeHandle n;
    getParam();

    enable_sub = n.subscribe("/enable",1,enable_callback);
    camera_sub = n.subscribe("/drag_race_cv/lane_tracker", 3, camera_callback);    
    velocity_sub = n.subscribe("/encoderVelocity", 3, velocity_encoder_callback);  
    steering_publisher = n.advertise<std_msgs::Float32>("/RR1/steering_cmd", 1);
    velocity_publisher = n.advertise<std_msgs::Float32>("/RR1/velocity_cmd", 1);

    ROS_INFO("Robot Racing Drag Race Controller: Intialized");
    
    while (ros::ok())
    {   
        //loop_rate.sleep();
        ros::spinOnce();
    } 
    ROS_INFO("Robot Racing Drag Race Controller: Task Completed");
    return 0;
}
