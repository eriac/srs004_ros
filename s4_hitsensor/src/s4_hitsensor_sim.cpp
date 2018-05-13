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

float light[2]={0};
void light0_callback(const std_msgs::Float32& float_msg){
	light[0]=float_msg.data;
}
void light1_callback(const std_msgs::Float32& float_msg){
	light[1]=float_msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phycon_hitsensor_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//publish
	ros::Publisher hit_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/hit", 1000);

	//Subscribe
	ros::Subscriber light0_sub     = n.subscribe(ros::this_node::getName()+"/light0", 10, light0_callback);
	ros::Subscriber light1_sub     = n.subscribe(ros::this_node::getName()+"/light1", 10, light1_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("HitSensorModule");
	updater.add("Connection", diagnostic0);
		
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish hit
		static int i=0;
		i++; 
		
		std_msgs::Float32 float_data;
		if(i%40==0)float_data.data=1.0;
		else float_data.data=0.0;
		hit_pub.publish(float_data);

		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

