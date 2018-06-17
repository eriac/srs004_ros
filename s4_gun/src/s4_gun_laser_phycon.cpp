#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "s4_comport/CANCode.h"

ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;

void laser_callback(const std_msgs::Bool& bool_msg){
	s4_comport::CANCode cancode;
	cancode.channel=CAN_CH;
	cancode.id=CAN_ID;
	cancode.com=3;
	cancode.length=1;
	if(bool_msg.data)cancode.data[0]=1;
	else             cancode.data[0]=0;
	canlink_pub.publish(cancode);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "pycon_gun_laser_phycon");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("CAN_CH", CAN_CH);
	pn.getParam("CAN_ID", CAN_ID);

	//publish
	canlink_pub = n.advertise<s4_comport::CANCode>("CANLink_out", 1000);

	//subscriibe
	ros::Subscriber laser_sub  = n.subscribe("laser", 10, laser_callback);

	ros::Duration(1.0).sleep();	
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

