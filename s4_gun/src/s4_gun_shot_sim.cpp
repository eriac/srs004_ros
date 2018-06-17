#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <diagnostic_updater/diagnostic_updater.h>

#include "math.h"
#include <string>


int   gun_shot=0;
void shot_callback(const std_msgs::Int32& int_msg){
	gun_shot=int_msg.data;
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Simulation.");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_shot_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher remain_pub = n.advertise<std_msgs::Float32>("remain", 10);

	//Subscribe
	ros::Subscriber shot_sub  = n.subscribe("shot", 10, shot_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("GunModule");
	updater.add("Active", diagnostic0);
		
	ros::Rate loop_rate(10); 
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
		remain_pub.publish(float_data);

		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}