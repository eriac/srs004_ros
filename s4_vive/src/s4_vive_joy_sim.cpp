#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <tf/tf.h>

#include <string>
#include "simple_udp.h"

simple_udp udp0;
std::string udp_address="127.0.0.1";
int udp_port=4001;
std::string selector="ALL";


std::string device_index="device1";
float input_axes[4]={0.0};
bool input_buttons[4]={false};

void joy_callback(const sensor_msgs::Joy& joy_msg){
	static std::string selected="ALL";
	if     (joy_msg.buttons[10])selected="L1";
	else if(joy_msg.buttons[8]) selected="L2";
	else if(joy_msg.buttons[11])selected="R1";
	else if(joy_msg.buttons[9]) selected="R2";

	if(selector==selected || selector=="ALL"){
		input_axes[0]=joy_msg.axes[0];
		input_axes[1]=joy_msg.axes[1];
		input_axes[2]=joy_msg.axes[2];
		input_axes[3]=joy_msg.axes[3];
		input_buttons[0]=joy_msg.buttons[12]*2;
		input_buttons[1]=joy_msg.buttons[13]*2;
		input_buttons[2]=joy_msg.buttons[14];
		input_buttons[3]=joy_msg.buttons[15];
		
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_vive_joy_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//rosparam
	pn.getParam("device_index", device_index);
	pn.getParam("udp_address",  udp_address);
	pn.getParam("udp_port",     udp_port);
	pn.getParam("selector",     selector);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

	udp0.udp_init(udp_address,udp_port);
	
	ros::Rate loop_rate(10);
	while (ros::ok()){
		//simulate
		static float pose_xyz[3]={0.0, 0.0, 0.0};
		static float pose_rpy[3]={0.0, 0.0, 0.0};
		/*
		bool buttons[5]={false};//trigger,pad,menu,grip
		bool pad_touch=false;
		float pad_position[2]={0,0};
		*/

		//xyz
		pose_xyz[0]+=input_axes[1]*0.1;
		pose_xyz[1]+=input_axes[0]*0.1;
		pose_xyz[2]=1.0;
		//quat
		pose_rpy[0]=0.0;
		pose_rpy[1]+=input_axes[3]*0.1;
		pose_rpy[2]+=input_axes[2]*0.1;
		tf::Quaternion tf_quat=tf::createQuaternionFromRPY(pose_rpy[0],pose_rpy[1],pose_rpy[2]);
		geometry_msgs::Quaternion geo_quat;
		quaternionTFToMsg(tf_quat,geo_quat);
		//send
		char buf[100];
		sprintf(buf,"C,%s,%f,%f,%f,%f,%f,%f,%f,%i,%i,%i,%i\n",
			device_index.c_str(),
			pose_xyz[0],
			pose_xyz[1],
			pose_xyz[2],
			geo_quat.x,
			geo_quat.y,
			geo_quat.z,
			geo_quat.w,
			input_buttons[0],
			input_buttons[1],
			input_buttons[2],
			input_buttons[3]
		);
		udp0.udp_send(std::string(buf));
		ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
