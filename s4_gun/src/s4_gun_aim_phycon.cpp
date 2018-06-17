#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "s4_comport/CANCode.h"

ros::Publisher canlink_pub;
ros::Publisher joint_pub;
std::string CAN_CH="";
int CAN_ID=0;
float z_ajust=0;
float y_ajust=0;

void command_callback(const geometry_msgs::Pose& command_msg){
	int servo1=1520+(command_msg.orientation.z+z_ajust)/(2*3.14)*360*10;
	int servo2=1520-(command_msg.orientation.y+y_ajust)/(2*3.14)*360*10;

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

int main(int argc, char **argv){
	ros::init(argc, argv, "pycon_gun_aim_phycon");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);
	pn.getParam("y_ajust", y_ajust);
	pn.getParam("z_ajust", z_ajust);

	//publish
	canlink_pub = n.advertise<s4_comport::CANCode>("CANLink_out", 10);

	//subscriibe
	ros::Subscriber command_sub  = n.subscribe("aim_command", 10, command_callback);

	ros::Duration(1.0).sleep();
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
