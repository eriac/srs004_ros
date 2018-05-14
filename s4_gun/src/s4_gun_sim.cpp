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

geometry_msgs::Twist aim_pos;
void pos_callback(const geometry_msgs::Twist& twist_msg){
	aim_pos=twist_msg;
}

int   gun_shot=0;
void shot_callback(const std_msgs::Int32& int_msg){
	gun_shot=int_msg.data;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher status_pub = n.advertise<std_msgs::Float32>("status", 1000);

	//Subscribe
	ros::Subscriber pos_sub  = n.subscribe("aim_pos", 10, pos_callback);
	ros::Subscriber shot_sub  = n.subscribe("shot", 10, shot_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("GunModule");
	updater.add("Connection", diagnostic0);
		
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish shot
		if(gun_shot>0){
			static int timer=0;
			if(timer%2==0){
				gun_shot--;
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

