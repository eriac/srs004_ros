#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
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

ros::Publisher vel_pub;
ros::Publisher pos_pub;
ros::Publisher laser_pub;
ros::Publisher shot_pub;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	geometry_msgs::Twist gun_vel;
	gun_vel.angular.y=joy_msg.axes[1];
	gun_vel.angular.z=joy_msg.axes[0];
	vel_pub.publish(gun_vel);

	if(joy_msg.buttons[PS3_cross]){
		std_msgs::Int32 int_data;
		int_data.data=1;
		shot_pub.publish(int_data);
	}
	else if(joy_msg.buttons[PS3_square]){
		std_msgs::Int32 int_data;
		int_data.data=5;
		shot_pub.publish(int_data);
	}
	else if(joy_msg.buttons[PS3_circle]){
		std_msgs::Bool bool_data;
		bool_data.data=true;
		laser_pub.publish(bool_data);
	}
	else if(joy_msg.buttons[PS3_triangle]){
		std_msgs::Bool bool_data;
		bool_data.data=false;
		laser_pub.publish(bool_data);
	}

	if(joy_msg.buttons[PS3_Start]){
		geometry_msgs::Pose aim_pose;
		pos_pub.publish(aim_pose);
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
    vel_pub   = n.advertise<geometry_msgs::Twist>("aim_vel", 10);
    pos_pub   = n.advertise<geometry_msgs::Pose>("aim_pos", 10);
    laser_pub = n.advertise<std_msgs::Bool>("laser", 10);
    shot_pub  = n.advertise<std_msgs::Int32>("shot", 10);
	
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
