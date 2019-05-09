#include "ros/ros.h"
  
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#include "math.h"
#include <sstream>
#include <string>

ros::Publisher wheel0_pub;
ros::Publisher wheel1_pub;
ros::Publisher wheel2_pub;

float wheel_base=0.100;
float wheel_radius=0.20;
float publish_rate=20;
float wheel[3]={M_PI/3, M_PI, 5*M_PI/3};
float wheel_normal[3];

void calculation(float *out, float *in){
	for(int i=0;i<3;i++){
		out[i]=(cos(wheel_normal[i])*in[0]+sin(wheel_normal[i])*in[1])/wheel_radius;
		out[i]+=-in[2]*wheel_base/wheel_radius;
	}
}

void twist_callback(const geometry_msgs::Twist& twist_msg){
	float in[3]={0};
	float out[3]={0};
	in[0]=twist_msg.linear.x;
	in[1]=twist_msg.linear.y;
	in[2]=twist_msg.angular.z;
	calculation(out,in);
	std_msgs::Float64 data[3];
	data[0].data=out[0];
	data[1].data=out[1];
	data[2].data=out[2];
	wheel0_pub.publish(data[0]);
	wheel1_pub.publish(data[1]);
	wheel2_pub.publish(data[2]);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_omni_twist");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//rosparam
	pn.getParam("wheel_base",    wheel_base);
	pn.getParam("wheel_radius",  wheel_radius);
	pn.getParam("publish_rate", publish_rate);
	pn.getParam("wheel0", wheel[0]);
	pn.getParam("wheel1", wheel[1]);
	pn.getParam("wheel2", wheel[2]);

	//publish
	wheel0_pub = n.advertise<std_msgs::Float64>("wheel0/command", 10);
	wheel1_pub = n.advertise<std_msgs::Float64>("wheel1/command", 10);
	wheel2_pub = n.advertise<std_msgs::Float64>("wheel2/command", 10);
	//Subscribe
	ros::Subscriber joy_sub     = n.subscribe("cmd_vel", 1, twist_callback); 

	for(int i=0;i<3;i++)wheel_normal[i]=wheel[i]-M_PI/2;
	
	ros::Rate loop_rate(publish_rate); 
  ros::spin();
 	return 0;
}

