#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "s4_comport/CANCode.h"
#include <diagnostic_updater/diagnostic_updater.h>

#include "math.h"
#include <sstream>
#include <string>

ros::Publisher voltage_pub;
ros::Publisher current_pub;
ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;

void canin_callback(const s4_comport::CANCode& can_msg){
	if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
		if(can_msg.com==1 /*&&can_msg.length==4*/){
			float data0=can_msg.data[0]+can_msg.data[1]/256.0;
			float data1=can_msg.data[2]+can_msg.data[3]/256.0;
			
			//publish voltage current
			std_msgs::Float32 float_data;
			float_data.data=data0;
			voltage_pub.publish(float_data);
			float_data.data=data1;
			current_pub.publish(float_data);
		}
	}
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phycon_master_actual");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);
	

	//publish
	voltage_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/voltage", 1000);
	current_pub = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/current", 1000);
	canlink_pub  = n.advertise<s4_comport::CANCode>("CANLink_out", 1000);

	//Subscribe
	ros::Subscriber canin_sub     = n.subscribe("CANLink_in", 10, canin_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("MainModule");
	updater.add("Active", diagnostic0);

	float dt=1.0/20;
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		s4_comport::CANCode can_msg;
		can_msg.channel=CAN_CH;
		can_msg.id=CAN_ID;
		can_msg.com=1;
		can_msg.remote=true;
		canlink_pub.publish(can_msg);
		
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
