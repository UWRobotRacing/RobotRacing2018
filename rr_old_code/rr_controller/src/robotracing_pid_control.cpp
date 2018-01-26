// Robot Racer Controller (Long and Lat)
// Sirui Song
// Date:04/07/14
//#define TESTOUTPUT

#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <vector>
#include <cmath>
#include <iostream>

#define PI 3.14159

double x;
double y;
double yaw;

//Controller Gains
double Kp_lat;
double Kd_lat;
double Ki_lat;
double lookahead;

double Kp_long;
double Kd_long;
double Ki_long;
double Kf_long;

//Error terms
double vel_error = 0;
double de_vel_error = 0;
double in_vel_error = 0;
double vel_desired = 1.5;

double heading_error = 0;
double dheading_error = 0;
double iheading_error = 0;

//Timing variable
double dt_ips = 0.01;
double dt_encoder = 0.01;
bool enable = true;

//Desired Path 
double *Waypts_xcircle;
double *Waypts_ycircle;

double x_0 = 0;
double y_0 = 0;
double x_1 = 0;
double y_1 = 0;


std_msgs::Float32 cmd_steering_msg;
std_msgs::Float32 cmd_throttle_msg;

ros::Subscriber pose_sub;
ros::Subscriber velocity_sub;

ros::Publisher steering_publisher;
ros::Publisher velocity_publisher;


using namespace std;

void getParam (ros::NodeHandle nh)
{
    nh.param<double>("Param_Kp_lat", Kp_lat, 1);
    nh.param<double>("Param_Kd_lat", Kd_lat, 0.0);
    nh.param<double>("Param_Ki_lat", Ki_lat, 0);
    nh.param<double>("Param_look_ahead", lookahead, 0.2);

    nh.param<double>("Param_Kp_long", Kp_long, 1);
    nh.param<double>("Param_Kd_long", Kd_long, 0.0);
    nh.param<double>("Param_Ki_long", Ki_long, 0);
    nh.param<double>("Param_Kf_long", Kf_long, 1);
    nh.param<double>("Vel_Desired", vel_desired, 2);
}

double time_elapsed(int ID){
//Calculates how much time has passed between msgs. 

    static ros::Time prev_t[2]= {ros::Time::now(), ros::Time::now()}; 
    if (ros::ok()){

        double timediff = (ros::Time::now() - prev_t[ID]).toNSec()* 1E-3;
        prev_t[ID] = ros::Time::now();
        return timediff*1E-6;
    }       
    return -1;
}

double PID (double error, double derror, double ierror, double desired, 
            double Kp,    double Kd,     double Ki,     double Kf){

    return Kp*error + Kd*derror + Ki*ierror + Kf * desired;
}

void long_error_calculation (double vel_measured, double dt){

    //To avoid integrator wind up, the enable pin should be tied to the Estop status 
    if (enable && dt > 0)
    {
        //calculate error
        double tmp_vel_error = vel_desired - vel_measured;
        de_vel_error = (vel_error-tmp_vel_error)/dt;
        in_vel_error = in_vel_error + tmp_vel_error;
        vel_error = tmp_vel_error;
       
        //call PID
        cmd_throttle_msg.data = PID(vel_error, de_vel_error, in_vel_error, vel_desired,
                                    Kp_long, Kd_long, Ki_long, Kf_long);
    }
}

double lateral_error_calculation (double dt){
    
    double roadheading = atan2((y_1-y_0),(x_1-x_0));
    double CTE = (sin(-roadheading)*(x-x_0) + cos(-roadheading)*(y-y_0)); 

    double errorheading= roadheading - yaw;
    double yawdesired= atan(CTE/lookahead);
    double yawerror = errorheading - yawdesired;

    std::cout << "Road heading = " << roadheading << std::endl;
    std::cout << "yawdesired = " << yawdesired << std::endl; 
    std::cout << "CTE = " << CTE << std::endl;
    std::cout << "errorHeading = " << errorheading << std::endl;


    if (yawerror > PI)
    {
        yawerror = 2*PI - yawerror;
    }
    else if (yawerror < -PI)
    {
        yawerror = 2*PI + yawerror;
    }  

    std::cout << "yawError after wrap = " << yawerror << std::endl;
    //To avoid integrator wind up, the enable pin should be tied to the Estop status 
    if (enable && dt > 0)
    {    
        dheading_error = (heading_error-yawerror)/dt;
        iheading_error = in_vel_error + yawerror;
        heading_error = yawerror;

        cmd_steering_msg.data = (float)(PID(heading_error, dheading_error, iheading_error, 0,
                                    Kp_lat, Kd_lat, Ki_lat, 0));
    }

}

void velocity_encoder_callback(const geometry_msgs::Point32 vel_e_msg){

    //This function should be changed to be receiving msg from estimators once fixed
    dt_encoder = time_elapsed(0);    
    long_error_calculation(vel_e_msg.y, dt_encoder);
    
    velocity_publisher.publish(cmd_throttle_msg);
}

void pose_callback(const geometry_msgs::Pose2D pose_msg){
    //This function should be changed to be receiving msg from estimators once fixed

    x = pose_msg.x;
    y = pose_msg.y;
    yaw = pose_msg.theta; 

    dt_ips = time_elapsed(1);    
    lateral_error_calculation(dt_ips);
    
    //Publish the msgs. 
    steering_publisher.publish(cmd_steering_msg);
    
}

void makeacircle(double x_circle,double y_circle, double radius, int numpointscircle)
{   
    Waypts_ycircle = new double [numpointscircle];
    Waypts_xcircle = new double [numpointscircle];

    double angle = 0;

    for (int i = 0; i < (numpointscircle); i++) 
    {
            Waypts_xcircle[i] = radius * cos(angle) + x_circle;
            Waypts_ycircle[i] = radius * sin(angle) + y_circle;
            angle += 2*PI/(numpointscircle-1);
    }
}


/* Do not need these anymore, add them to the arduino code instead
int velocity2Pulses (double velocity){
    return (int)((velocity+169.038)/0.1122);
}

int angle2Pulses (double angle){
    //angle in radians
    int output = (int) (-450*(angle)/(25*PI/180)+1500);
    output = std::max(1000, output);
    output = std::min(2000, output);
    return output;
}
*/

int main(int argc, char **argv)
{
    //INITIALIZATION
    ros::init(argc,argv,"robotracer_PID_controller");
    ros::NodeHandle n;
    getParam(n);

    pose_sub = n.subscribe("/RR1/ground_pose", 3, pose_callback);
    velocity_sub = n.subscribe("/encoderVelocity", 3, velocity_encoder_callback);  
    steering_publisher = n.advertise<std_msgs::Float32>("/RR1/steering_cmd", 1);
    velocity_publisher = n.advertise<std_msgs::Float32>("/RR1/velocity_cmd", 1);

    //Set trajectory  
    int wp_index = 0;
    int numPoints = 25;
    
    makeacircle(0.4,-0.8,0.9,numPoints);

    while (ros::ok())
    {   
        //loop_rate.sleep();
        ros::spinOnce();

        x_0 = Waypts_xcircle[wp_index];
        y_0 = Waypts_ycircle[wp_index];
        x_1 = Waypts_xcircle[wp_index+1];
        y_1 = Waypts_ycircle[wp_index+1];
        
        #ifdef TESTOUTPUT
            std::cout << "Current Position is x = " << x << ", y = " << y << std::endl;
            std::cout << "Current Target is x = " << x_1 << ", y = " << y_1 << std::endl;
	    std::cout << "error Heading is " << heading_error << std::endl;
            std::cout << "Steering output is " << cmd_steering_msg.data << std::endl;
        #endif
        
        if (sqrt(pow(x-x_1,2)+pow(y-y_1,2)) <= 0.25)
        {
            wp_index++;

            if (wp_index >= numPoints)
                wp_index = 0;
        } 
    } 
    ROS_INFO("Robot Racing Controller: Task Completed");
    return 0;
}
