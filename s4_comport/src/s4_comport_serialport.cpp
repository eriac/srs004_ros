#include "ros/ros.h"
#include "std_msgs/String.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int open_serial(const char *device_name){
	int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(fd1, F_SETFL,0);
	//load configuration
	struct termios conf_tio;
	tcgetattr(fd1,&conf_tio);
	//set baudrate
	speed_t BAUDRATE = B1000000;
	cfsetispeed(&conf_tio, BAUDRATE);
	cfsetospeed(&conf_tio, BAUDRATE);
	//non canonical, non echo back
	conf_tio.c_lflag &= ~(ECHO | ICANON);
	//non blocking
	conf_tio.c_cc[VMIN]=0;
	conf_tio.c_cc[VTIME]=0;
	//store configuration
	tcsetattr(fd1,TCSANOW,&conf_tio);
	return fd1;
}

int fd1;
int diagnostic_counter=0;
bool diagnostic_connect=false;
void serial_callback(const std_msgs::String& serial_msg){
	int rec=write(fd1,serial_msg.data.c_str(),serial_msg.data.size());
	if(rec>=0)printf("send:%s\n",serial_msg.data.c_str());
	else{
		diagnostic_connect=false;
		ROS_ERROR_ONCE("Serial Fail: cound not write");
	}
}
void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(diagnostic_connect){
		if(diagnostic_counter<100){
			stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
		}
		else{
			stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Not Active.");
		}
	}
	else{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_comport_serialport");
	ros::NodeHandle n;
	
	//Publisher
	ros::Publisher serial_pub = n.advertise<std_msgs::String>("Serial_in", 1000);

	//Subscriber
	ros::Subscriber serial_sub = n.subscribe("Serial_out", 10, serial_callback); 

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("SerialPort");
	updater.add("Connect", diagnostic0);
	
	char device_name[]="/dev/ttyUSB0";
	fd1=open_serial(device_name);
	if(fd1<0){
		ROS_ERROR("Serial Fail: cound not open %s", device_name);
		printf("Serial Fail\n");
		ROS_BREAK();
	}
	else diagnostic_connect=true;

	ros::Rate loop_rate(200); 
	while (ros::ok()){
		char buf[256]={0};
		int recv_data=read(fd1, buf, sizeof(buf));
		if(recv_data>0){
			printf("recv:%03d %s\n",recv_data,buf);
			std_msgs::String serial_msg;
			serial_msg.data=buf;
			serial_pub.publish(serial_msg);
			diagnostic_counter=0;
		}
		updater.update();		
		diagnostic_counter++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

