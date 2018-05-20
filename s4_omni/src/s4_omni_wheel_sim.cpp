#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <diagnostic_updater/diagnostic_updater.h>

#include "math.h"
#include <sstream>
#include <string>


float wheel_target=0.0;
void target_callback(const std_msgs::Float32& float_msg){
	wheel_target=float_msg.data;
}
float wheel_odometry=0.0;
float wheel_current=0.0;

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Simulation.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phycon_wheel_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher current_pub = n.advertise<std_msgs::Float32>("current", 1000);
	ros::Publisher odometry_pub = n.advertise<std_msgs::Float32>("odometry", 1000);

	//Subscribe
	ros::Subscriber target_sub = n.subscribe("target", 10, target_callback); 
	
	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("WheelModule");
	updater.add("Connection", diagnostic0);


	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish current
		float wheel_value=0.0;
		static float wheel_value_last=0.0;
		float current=0.0;
		
		float acc=wheel_target-wheel_value_last;
		float acc_limit=100.0;
		if     (acc < -acc_limit*dt)wheel_value=wheel_value_last-acc_limit*dt;
		else if(acc > +acc_limit*dt)wheel_value=wheel_value_last+acc_limit*dt;
		else wheel_value=wheel_target;

		current=1.0*(fabsf(wheel_value)-fabsf(wheel_value_last));
		current+=0.03*fabsf(wheel_value);
		if(current<0)current=0;
		wheel_value_last=wheel_value;
		std_msgs::Float32 float_data;
		float_data.data=current;
		current_pub.publish(float_data);
		wheel_current=current;

		//publish odometry
		wheel_odometry+=wheel_value*dt;
		
		std_msgs::Float32 float_data2;
		float_data2.data=wheel_odometry;
		odometry_pub.publish(float_data2);
		
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

