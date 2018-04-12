#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
static volatile state;

EndlineCounter::EndlineCounter(ros::NodeHandle nh) : it(nh)
{
	
	nh.advertise<std_msgs::Int8>(endline,1);
	state = BEGINNING;
}

EndlineCounter::img_callback(const sensor_msgs::ImagePtr& msg) {
	image_transport::Publisher pub;
	cv_bridge::CvImagePtr first_img = cv_bridge::toCvCopy(msg, "bgr8")->image); 
	
	//process image
	
	
	
	
	
	
	//
	
	//publish 
}


int main(int argc, char** argv)
{
	
	ros::init(argc,argv, "endline_counter");
	ros::NodeHandle nh;
	std::cout << "Initializing endline_counter" << endl;
	
	//nodehandler.param ?
	
	image_transport::ImageTransport it;
	image_transport::Subscriber sub = it.subscribe("tl_camera/image",1, &EndlineCounter::img_callback);

	
	
	while(ros::ok)
	{
	
		
		ros::spinOnce();
	}

{
