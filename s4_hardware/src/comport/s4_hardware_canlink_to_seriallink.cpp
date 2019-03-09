#include "ros/ros.h"
#include "s4_hardware/CANCode.h"
#include "s4_hardware/SerialCode.h"

#include <sstream>
#include <string>

int cancode_to_serialcode(s4_hardware::SerialCode *serialcode, s4_hardware::CANCode cancode){
	char tmp_data[2];
	s4_hardware::SerialCode midcode;
	midcode.command[0]="CANLINK";
	midcode.command[1]=cancode.channel;
	midcode.option[0]="ID";
	sprintf(tmp_data,"%d",cancode.id);
	midcode.suboption[0]=std::string(tmp_data);
	midcode.option[1]="COM";
	sprintf(tmp_data,"%d",cancode.com);
	midcode.suboption[1]=std::string(tmp_data);
	if(cancode.remote)midcode.option[2]="REMOTE";
	for(int i=0;i<cancode.length;i++){
		midcode.data[i]=cancode.data[i];
	}
	midcode.datanum=cancode.length;
	*serialcode=midcode;
	return 0;
}

ros::Publisher seriallink_pub;
void canlink_callback(const s4_hardware::CANCode& cancode_msg){
	s4_hardware::SerialCode outdata;

	cancode_to_serialcode(&outdata,cancode_msg);
	seriallink_pub.publish(outdata);
	//printf("%s\n",outdata.c_str());
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_hardware_canlink_to_seriallink");
	ros::NodeHandle n;
	//publicher
	seriallink_pub = n.advertise<s4_hardware::SerialCode>("SerialLink_out", 1000);
	//Subscriber
	ros::Subscriber canlink_sub = n.subscribe("CANLink_out", 10, canlink_callback); 
	
	ros::Rate loop_rate(100); 
	while (ros::ok()){ 		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

