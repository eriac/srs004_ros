#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <string>
#include "math.h"

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

ros::Publisher aim_pub;
ros::Publisher cmd_pub;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	geometry_msgs::Twist gun_vel;
	gun_vel.angular.y=joy_msg.axes[1];
	gun_vel.angular.z=joy_msg.axes[0];
	aim_pub.publish(gun_vel);

	std_msgs::String command;
	if(joy_msg.buttons[PS3_cross]){
		command.data="shot";
		cmd_pub.publish(command);
	}
	else if(joy_msg.buttons[PS3_circle]){
		command.data="laser_on";
		cmd_pub.publish(command);
	}
	else if(joy_msg.buttons[PS3_triangle]){
		command.data="laser_off";
		cmd_pub.publish(command);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_demo");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	//rosparam
	//pn.getParam("linear_velocity",  linear_velocity);
	//pn.getParam("angular_velocity", angular_velocity);

    //publish
    aim_pub = n.advertise<geometry_msgs::Twist>("aim_vel", 1000);
    cmd_pub = n.advertise<std_msgs::String>("command", 1000);
	
    //subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);
	
	float dt=1.0/20;
	ros::Rate loop_rate(20);

	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
