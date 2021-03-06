#include "ros/ros.h"
#include "s4_hardware/SerialCode.h"
#include "s4_hardware/CANCode.h"

#include <sstream>
#include <string>

int hex_convert(std::string input){
	int data_size=input.size();
	int output=0;
	if(data_size==0)return -1;
	for(int i=0;i<data_size;i++){
		if     ('0'<=input[i] && input[i]<='9')output|=(input[i]-'0')<<((data_size-i-1)*4);
		else if('A'<=input[i] && input[i]<='F')output|=(input[i]-'A'+10)<<((data_size-i-1)*4);
		else return -1;
	}
	return output;
}

ros::Publisher  canlink_pub;
void seriallink_callback(const s4_hardware::SerialCode& serialcode_msg){
	if(serialcode_msg.command[0]=="CANLINK"){
		printf("CANLINK_conv\n");
		s4_hardware::CANCode outdata;
		outdata.channel=serialcode_msg.command[1];
		outdata.id=0;//default?
		outdata.com=0;//default?
		outdata.remote=false;
		for(int i=0;i<4;i++){
			if(serialcode_msg.option[i]=="ID"){
				int ret=hex_convert(serialcode_msg.suboption[i]);
				if(0<=ret && ret<=15)outdata.id=ret;
			}
			else if(serialcode_msg.option[i]=="COM"){
				int ret=hex_convert(serialcode_msg.suboption[i]);
				if(0<=ret && ret<=15)outdata.com=ret;
			}
			else if(serialcode_msg.option[i]=="REMOTE"){
				outdata.remote=true;
			}
		}
		int data_size=serialcode_msg.datanum;
		if(data_size>8)data_size=8;
		for(int i=0;i<data_size;i++){
			outdata.data[i]=serialcode_msg.data[i];
		}
		outdata.length=data_size;
		canlink_pub.publish(outdata);
	}
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_hardware_seriallink_to_canlink");
	ros::NodeHandle n;
	//publicher
	canlink_pub = n.advertise<s4_hardware::CANCode>("CANLink_in", 10);
	//Subscriber
	ros::Subscriber seriallink_sub = n.subscribe("SerialLink_in", 10, seriallink_callback); 
	
	ros::spin();
 	return 0;
}

