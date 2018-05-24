#include "ros/ros.h"
  
#include "math.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <string>
#include <iostream>
#include <sstream>

#define PS3_Button_Max 17
#define PS3_Select 0
#define PS3_L3     1
#define PS3_R3     2
#define PS3_Start  3
#define PS3_Up     4
#define PS3_Right  5
#define PS3_Down   6
#define PS3_Left   7
#define PS3_L2     8
#define PS3_R2     9
#define PS3_L1    10
#define PS3_R1    11
#define PS3_triangle 12
#define PS3_circle   13
#define PS3_cross    14
#define PS3_square   15
#define PS3_PS    16

float input[6]={0};
ros::Publisher move_twist_pub;
ros::Publisher gun_twist_pub;
ros::Publisher gun_command_pub;
ros::Publisher zoom_pub;
ros::Publisher light_pub;
ros::Publisher hit_reset_pub;

void joy_callback(const sensor_msgs::Joy& joy_msg){
	bool b_push[PS3_Button_Max]={0};
	bool b_release[PS3_Button_Max]={0};
	static bool b_last[PS3_Button_Max]={0};
	for(int i=0;i<PS3_Button_Max;i++){
		if(joy_msg.buttons[i] && !b_last[i])b_push[i]=true;
		else b_push[i]=false;
		if(!joy_msg.buttons[i] && b_last[i])b_release[i]=true;
		else b_release[i]=false;
		b_last[i]=joy_msg.buttons[i];
	}	

	//move
	geometry_msgs::Twist twist_data;
	float linear_gain=0.2;
	float angular_gain=0.6;
	if(joy_msg.buttons[PS3_L3]){
		linear_gain=1.0;
		angular_gain=1.0;
	}
	if(!joy_msg.buttons[PS3_L1]){
		twist_data.linear.x =linear_gain* joy_msg.axes[1];
		twist_data.angular.z=angular_gain*joy_msg.axes[0];
	}
	else{
		twist_data.linear.x =linear_gain* joy_msg.axes[1];
		twist_data.linear.y =linear_gain*joy_msg.axes[0];
	}
	move_twist_pub.publish(twist_data);


	//gun
	float gun_gain=1.0;
	if(joy_msg.buttons[PS3_R1])gun_gain=0.1;
	geometry_msgs::Twist twist_gdata;
	twist_gdata.angular.z =gun_gain*joy_msg.axes[2];
	twist_gdata.angular.y =gun_gain*joy_msg.axes[3];
	gun_twist_pub.publish(twist_gdata);

	if(joy_msg.buttons[PS3_R1] && b_push[PS3_cross]){
		std_msgs::String command_data;
		command_data.data="shot";
		gun_command_pub.publish(command_data);
	}

	if(b_push[PS3_R1]){
		std_msgs::String command_data;
		command_data.data="laser_on";
		gun_command_pub.publish(command_data);
	}	
	if(b_push[PS3_Right]){
		std_msgs::String command_data;
		command_data.data="laser_off";
		gun_command_pub.publish(command_data);
	}	

	//camera
	if(b_push[PS3_Up]){
		std_msgs::Int32 int_data;
		int_data.data=4;
		zoom_pub.publish(int_data);
	}
	if(b_push[PS3_Left]){
		std_msgs::Int32 int_data;
		int_data.data=2;
		zoom_pub.publish(int_data);
	}
	if(b_push[PS3_Down]){
		std_msgs::Int32 int_data;
		int_data.data=1;
		zoom_pub.publish(int_data);
	}

	//light
	if(b_push[PS3_triangle]){
		static bool on_off=false;
		on_off=!on_off;
		std_msgs::Float32 command_data;
		if(on_off)command_data.data=0.3;
		else command_data.data=0.0;
		light_pub.publish(command_data);
	}

	//other
	if(b_push[PS3_Start]){
		std_msgs::Float32 command_data;
		command_data.data=0.0;
		hit_reset_pub.publish(command_data);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_operate_manual");
	ros::NodeHandle n;
	
	move_twist_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
	gun_twist_pub   = n.advertise<geometry_msgs::Twist>("aim_vel", 1000); 
	gun_command_pub = n.advertise<std_msgs::String>("gun_command", 1000); 
	zoom_pub        = n.advertise<std_msgs::Int32>("camera_zoom", 1000);
	light_pub       = n.advertise<std_msgs::Float32>("light", 1000);
	hit_reset_pub   = n.advertise<std_msgs::Float32>("hit_reset", 1000);

	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback); 

	ros::Rate loop_rate(50); 
	while (ros::ok()){		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

