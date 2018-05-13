#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "math.h"
#include <sstream>
#include <string>

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Simulation.");
}

float gun_yaw=0;
float gun_tilt=0;
int   gun_shot=0;
std::string gun_command="";
int   gun_sum=0;

void yaw_callback(const std_msgs::Float32& float_msg){
	gun_yaw=float_msg.data;
}
void tilt_callback(const std_msgs::Float32& float_msg){
	gun_tilt=float_msg.data;
}
void shot_callback(const std_msgs::Int32& int_msg){
	gun_shot=int_msg.data;
}
void command_callback(const std_msgs::String& string_msg){
	gun_command=string_msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phycon_gun_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//publish
	ros::Publisher status_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/status", 1000);

	//Subscribe
	ros::Subscriber gun0_sub     = n.subscribe(ros::this_node::getName()+"/yaw", 10, yaw_callback);
	ros::Subscriber gun1_sub     = n.subscribe(ros::this_node::getName()+"/pitch", 10, tilt_callback);
	ros::Subscriber gun2_sub     = n.subscribe(ros::this_node::getName()+"/shot", 10, shot_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("WheelModule");
	updater.add("Connection", diagnostic0);
		
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish shot
		if(gun_shot>0){
			static int timer=0;
			if(timer%2==0){
				gun_shot--;
				gun_sum++;
			}
			timer++;
		}
		
		std_msgs::Float32 float_data;
		float_data.data=gun_shot;
		status_pub.publish(float_data);

		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

