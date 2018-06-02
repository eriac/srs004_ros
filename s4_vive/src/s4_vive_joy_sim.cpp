#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <tf/tf.h>

#include "math.h"
#include <string>
#include "simple_udp.h"
#include "ps3_config.h"

simple_udp udp0;
std::string udp_address="127.0.0.1";
int udp_port=4001;
std::string selector="ALL";


std::string device_index="device1";
float input_axes[6]={0.0};
bool input_buttons[4]={false};

void joy_callback(const sensor_msgs::Joy& joy_msg){
	static std::string selected="ALL";
	if     (joy_msg.buttons[PS3_L1])selected="L1";
	else if(joy_msg.buttons[PS3_L2])selected="L2";
	else if(joy_msg.buttons[PS3_R1])selected="R1";
	else if(joy_msg.buttons[PS3_R2])selected="R2";

	if(selector==selected || selector=="ALL"){
		input_axes[0]=joy_msg.axes[PS3_LX];
		input_axes[1]=joy_msg.axes[PS3_LY];
		input_axes[2]=joy_msg.axes[PS3_RX];
		input_axes[3]=joy_msg.axes[PS3_RY];
		
		if(joy_msg.buttons[PS3_Up])       input_axes[4]=+1.0;
		else if(joy_msg.buttons[PS3_Down])input_axes[4]=-1.0;
		else input_axes[4]=0.0;
		
		if(joy_msg.buttons[PS3_Left])      input_axes[5]=+1.0;
		else if(joy_msg.buttons[PS3_Right])input_axes[5]=-1.0;
		else input_axes[5]=0.0;

		input_buttons[0]=joy_msg.buttons[PS3_triangle]*2;
		input_buttons[1]=joy_msg.buttons[PS3_circle]*2;
		input_buttons[2]=joy_msg.buttons[PS3_cross];
		input_buttons[3]=joy_msg.buttons[PS3_square];
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
		static float pose_xyz[3]={0.0, 0.0, 1.0};
		static float pose_rpy[3]={0.0, 0.0, 0.0};
		/*
		bool buttons[5]={false};//trigger,pad,menu,grip
		bool pad_touch=false;
		float pad_position[2]={0,0};
		*/

		//xyz
		pose_xyz[0]+=input_axes[0]*0.1;
		pose_xyz[1]+=input_axes[1]*0.1;
		pose_xyz[2]+=input_axes[4]*0.02;
		//quat
		pose_rpy[0]+=input_axes[5]*0.05;
		pose_rpy[1]+=input_axes[2]*0.1;
		pose_rpy[2]+=input_axes[3]*0.1-input_axes[5]*0.07;
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
