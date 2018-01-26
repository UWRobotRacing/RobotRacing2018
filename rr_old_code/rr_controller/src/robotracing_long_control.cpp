// Robot Racer Controller
// Sirui Song
// Date:15/05/14

//This is a longitudinal controller. It takes in velocity (in m/s) from
//the Twist geometry msgs (point32 for now) provided by the EKF. It outputs
//the desired velocity in an Int16 msg. 
//The controller has two parts: feedback: PID, and feedforward. 


#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <iostream>

#define PI 3.14159
#define MAX_VEL 50
#define MIN_VEL 0

double vel_desired = 0;
double vel_measured = 0;

//Error terms: 
double vel_error = 0;
double de_vel_error = 0;
double in_vel_error = 0;

double dt = 0.01;
ros::Time prev_t; 
bool enable = true;

double Kp;
double Kd;
double Ki;
double Kf;

std_msgs::Float32 cmd_throttle_msg;

using namespace std;
//Velocity Callback

void error_calculation ();
void PID ();

double time_elapsed()
{
    //Calculates how much time has passed between msgs. 
    //assumes max elapsed time is 1 second;    
    if (ros::ok())
    {
               double timediff = (ros::Time::now() - prev_t).toNSec()* 1E-3;
               prev_t = ros::Time::now();
               return timediff*1E-6;
       }       
       return -1;

}

void velocity_desired_callback(const geometry_msgs::Twist vel_d_msg){
    vel_desired = vel_d_msg.linear.x;

    dt = time_elapsed();
    //Call error calculation
    error_calculation();
}

void velocity_encoder_callback(const geometry_msgs::Point32 vel_e_msg){
       //Using rear tires only 
       //This function should be changed to be receiving msg from estimators once fixed
       vel_measured = vel_e_msg.y;

    dt = time_elapsed();

       //Call error calculation
       error_calculation();
}

void status_callback(const std_msgs::Int8 status){
       if (status.data == 2)
       {
               enable = false;
       }
}

void error_calculation (){
       //Check for encoder windup!! 
       if (enable && dt > 0)
       {
               //calculate error
               double tmp_vel_error = vel_desired - vel_measured;
               de_vel_error = (vel_error-tmp_vel_error)/dt;
               in_vel_error = in_vel_error + tmp_vel_error;
               vel_error = tmp_vel_error;
       
               //call PID
               PID();

       }
}

void PID(){
       //fb
       double vel_out = Kp * vel_error + Ki * in_vel_error + Kd * de_vel_error;
       //ff
       vel_out = vel_out + Kf * vel_desired;

       if (vel_out > MAX_VEL)
       {
               vel_out = MAX_VEL;
       }
       if (vel_out < MIN_VEL)
       {
               vel_out = MIN_VEL;
       }

       //Publish messages
       cmd_throttle_msg.data = vel_out;
       
}

int main(int argc, char **argv)
{
       ros::init(argc,argv,"robotracing_long_control");
       ros::NodeHandle n;
       ros::NodeHandle private_nh("~");

    private_nh.param<double>("Param_Kp", Kp, 10);
    private_nh.param<double>("Param_Kd", Kd, 0.0);
    private_nh.param<double>("Param_Ki", Ki, 0);
    private_nh.param<double>("Param_Kf", Kf, 1);
    private_nh.param<double>("Vel_Desired", vel_desired, 2);
       
       ros::Subscriber velocity_e_subscriber = n.subscribe("/encoderVelocity", 3, velocity_encoder_callback);
       //ros::Subscriber velocity_d_subscriber = n.subscribe("Velocity_desired", 3, velocity_desired_callback);
       ros::Publisher velocity_publisher = n.advertise<std_msgs::Float32>("/RR1/velocity_cmd", 1);

       prev_t = ros::Time::now();

    while (ros::ok())
    {
       ROS_INFO("Ros inside ok");
       ros::spinOnce();
       velocity_publisher.publish(cmd_throttle_msg);
    }
    ROS_INFO("Robot Racing Longitudinal Controller: Task Completed");
    return 0;
}