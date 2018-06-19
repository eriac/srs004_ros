#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <diagnostic_updater/diagnostic_updater.h>

#include "math.h"
#include <sstream>
#include <string>

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Simulation.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_system_battery_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher voltage_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/voltage", 1000);
	ros::Publisher current_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/current", 1000);
	//Subscribe
	//ros::Subscriber wheel0_sub     = n.subscribe("/wheel0", 10, wheel0_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("MainModule");
	updater.add("Connection", diagnostic0);

	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish voltage current
		std_msgs::Float32 float_data;
		float_data.data=11.5;
		voltage_pub.publish(float_data);
		float_data.data=1.2;
		current_pub.publish(float_data);
		
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
