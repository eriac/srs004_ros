#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "s4_comport/CANCode.h"
#include "diagnostic_updater/diagnostic_updater.h"

ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;
float z_ajust=0;
float y_ajust=0;
void twist_callback(const geometry_msgs::Twist& twist_msg){
	int servo1=1520+(twist_msg.angular.z+z_ajust)/(2*3.14)*360*10;
	int servo2=1520-(twist_msg.angular.y+y_ajust)/(2*3.14)*360*10;

	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=1;
	cancode.length=4;
	cancode.data[0]=(servo1>>8)&0xFF;
	cancode.data[1]=(servo1>>0)&0xFF;
	cancode.data[2]=(servo2>>8)&0xFF;
	cancode.data[3]=(servo2>>0)&0xFF;
	canlink_pub.publish(cancode);
}
void shot_callback(const std_msgs::Int32& int_msg){
	int shot_num=int_msg.data;
	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=2;
	cancode.length=1;
	cancode.data[0]=shot_num;
	canlink_pub.publish(cancode);		
}
void laser_callback(const std_msgs::Int32& int_msg){
	int laser_num=int_msg.data;
	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=3;
	cancode.length=1;
	cancode.data[0]=laser_num;
	canlink_pub.publish(cancode);
}
int diagnostic_counter=0;
void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "No interaction.");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pycon_gun_actual");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);
	pn.getParam("y_ajust", y_ajust);
	pn.getParam("z_ajust", z_ajust);

	//publish
	canlink_pub = n.advertise<s4_comport::CANCode>("CANLink_out", 1000);

	//subscriibe
	ros::Subscriber twist_sub  = n.subscribe("aim_pos", 10, twist_callback);
	ros::Subscriber shot_sub   = n.subscribe("shot", 10, shot_callback);
	ros::Subscriber laser_sub  = n.subscribe("laser", 10, laser_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("GunModule");
	updater.add("Active", diagnostic0);

	ros::Duration(1.0).sleep();
	
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		updater.update();
		diagnostic_counter++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

