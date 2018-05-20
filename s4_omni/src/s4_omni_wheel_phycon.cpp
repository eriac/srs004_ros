#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "s4_comport/CANCode.h"
#include "diagnostic_updater/diagnostic_updater.h"

ros::Publisher  odometry_pub;
ros::Publisher  current_pub;
ros::Publisher  canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;
float PPR=400;
int diagnostic_counter=0;

void target_callback(const std_msgs::Float32& float_msg){
	float target=float_msg.data*PPR/(2*3.14*20);
	int data=0x1000+(target/2/3.1415*102.1)*0x1000/4000;
	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=1;
	cancode.length=2;
	cancode.data[0]=(data>>8)&0xFF;
	cancode.data[1]=(data>>0)&0xFF;
	canlink_pub.publish(cancode);
}

void canin_callback(const s4_comport::CANCode& can_msg){
	if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
		if(can_msg.com==1 /*&&can_msg.length==6*/){
			int temp1=can_msg.data[0]<<24|can_msg.data[1]<<16|can_msg.data[2]<<8|can_msg.data[3]<<0;
			int temp2=can_msg.data[4]<<8|can_msg.data[5]<<0;
			float o_data=(temp1-0x10000000)/PPR*2*3.1415;
			float c_data=(temp2-0x1000)/256.0;

			std_msgs::Float32 o_msg;
			o_msg.data=o_data;
			odometry_pub.publish(o_msg);
			std_msgs::Float32 c_msg;
			c_msg.data=c_data;
			current_pub.publish(c_msg);
		}
		diagnostic_counter=0;
	}
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(diagnostic_counter<10){
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
	}
	else{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Response.");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pycon_wheel_actual");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);
	pn.getParam("PPR", PPR);

	//publish
	odometry_pub = n.advertise<std_msgs::Float32>("odometry", 1000);
	current_pub  = n.advertise<std_msgs::Float32>("current", 1000);
	canlink_pub  = n.advertise<s4_comport::CANCode>("CANLink_out", 1000);

	//subscriibe
	ros::Subscriber target_sub = n.subscribe("target", 10, target_callback); 
	ros::Subscriber canin_sub  = n.subscribe("CANLink_in", 10, canin_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("WheelModule");
	updater.add("Active", diagnostic0);


	ros::Duration(1.0).sleep();
	
	ros::Rate loop_rate(20); 
	while (ros::ok()){

		s4_comport::CANCode cancode;
		cancode.channel=CAN_CH;
		cancode.id=CAN_ID;
		cancode.com=1;
		cancode.remote=true;
		canlink_pub.publish(cancode);
		
		updater.update();
		diagnostic_counter++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

