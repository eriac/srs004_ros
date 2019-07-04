#include "ros/ros.h"

#include "std_msgs/String.h"
#include "s4_hardware/SerialCode.h"

#include <sstream>
#include <string>

int serialcode_to_serial(std::string *outstring, s4_hardware::SerialCode serialcode){
	char tmp_data[2];
	std::string midstring="";
	midstring ="#"+serialcode.command[0];
	for(int i=1;i<4;i++){
		if(serialcode.command[i]!=""){
			midstring+="."+serialcode.command[i];
		}
		else break;
	}
	for(int i=0;i<4;i++){
		if(serialcode.option[i]!=""){
			midstring+="-"+serialcode.option[i];
			if(serialcode.suboption[i]!=""){
				midstring+="."+serialcode.suboption[i];
			}
		}
		else break;
	}
	if(serialcode.datanum!=0){
		midstring+=":";
		for(int i=0;i<serialcode.datanum;i++){
			sprintf(tmp_data,"%02X",serialcode.data[i]);
			midstring+=std::string(tmp_data);
		}
	}
	midstring+=";";
	*outstring=midstring;
	return 0;
}

ros::Publisher  serial_pub;
void seriallink_callback(const s4_hardware::SerialCode& cancode_msg){
	std::string outdata;

	serialcode_to_serial(&outdata,cancode_msg);
	std_msgs::String msg;
	msg.data=outdata;
	serial_pub.publish(msg);
	//printf("%s\n",outdata.c_str());
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_hardware_seriallink_decoder");
	ros::NodeHandle n;
	//publicher
	serial_pub = n.advertise<std_msgs::String>("Serial_out", 10);
	//Subscriber
	ros::Subscriber seriallink_sub = n.subscribe("SerialLink_out", 10, seriallink_callback); 
	
	ros::spin();
 	return 0;
}

