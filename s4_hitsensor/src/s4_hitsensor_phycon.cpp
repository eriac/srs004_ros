#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "s4_comport/CANCode.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <math.h>
#include <sstream>
#include <string>

ros::Publisher hit_pub;
ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;
int hit_counter=0;
void reset_callback(const std_msgs::Float32& float_msg){
	hit_counter=float_msg.data;
}
int diagnostic_counter=0;
void canlink_callback(const s4_comport::CANCode& can_msg){
	if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
		if(can_msg.com==1 /*&&can_msg.length==1*/){
			hit_counter=(int)hit_counter+can_msg.data[0];
			std_msgs::Float32 t_msg;
			t_msg.data=hit_counter;
			hit_pub.publish(t_msg);
		}
		diagnostic_counter=0;
	}
}
void light_callback(const std_msgs::Float32& float_msg){
	if(0<=float_msg.data && float_msg.data<=1){
		int data=float_msg.data*0xff;
		s4_comport::CANCode cancode;
		cancode.channel=CAN_CH;
		cancode.id=CAN_ID;
		cancode.com=2;
		cancode.length=1;
		cancode.data[0]=data;
		canlink_pub.publish(cancode);
	}
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(diagnostic_counter<10){
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
	}
	else{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Responce.");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phycon_hitsensor_actual");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);

	//publish
	hit_pub      = n.advertise<std_msgs::Float32>(ros::this_node::getName()+"/hit", 1000);
	canlink_pub  = n.advertise<s4_comport::CANCode>("CANLink_out", 1000);

	//Subscribe
	ros::Subscriber reset_sub   = n.subscribe(ros::this_node::getName()+"/reset", 10, reset_callback);
	ros::Subscriber light_sub   = n.subscribe(ros::this_node::getName()+"/light", 10, light_callback);
	ros::Subscriber canlink_sub = n.subscribe("CANLink_in", 10, canlink_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("HitSensorModule");
	updater.add("Active", diagnostic0);
		
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//com1 remote
		s4_comport::CANCode cancode;
		cancode.channel=CAN_CH;
		cancode.id=CAN_ID;
		cancode.com=1;
		cancode.length=1;
		cancode.remote=true;
		canlink_pub.publish(cancode);

		updater.update();
		diagnostic_counter++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

