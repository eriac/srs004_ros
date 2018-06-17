#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "s4_comport/CANCode.h"
#include "diagnostic_updater/diagnostic_updater.h"

ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;

void shot_callback(const std_msgs::Int32& int_msg){
	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=2;
	cancode.length=1;
	cancode.data[0]=int_msg.data;
	canlink_pub.publish(cancode);		
}

ros::Publisher remain_pub;
int diagnostic_counter=0;
void canin_callback(const s4_comport::CANCode& can_msg){
	if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
		if(can_msg.com==2){			
			std_msgs::Float32 float_msg;
			float_msg.data=can_msg.data[0];
			remain_pub.publish(float_msg);
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


int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_shot_phycon");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);

	//publish
	canlink_pub = n.advertise<s4_comport::CANCode>("CANLink_out", 10);
	remain_pub = n.advertise<std_msgs::Float32>("remain", 10);

	//subscriibe
	ros::Subscriber shot_sub   = n.subscribe("shot", 10, shot_callback);
	ros::Subscriber canin_sub  = n.subscribe("CANLink_in", 10, canin_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("GunModule");
	updater.add("Active", diagnostic0);

	ros::Duration(1.0).sleep();
	
	ros::Rate loop_rate(10);
	while (ros::ok()){
		updater.update();
		diagnostic_counter++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

