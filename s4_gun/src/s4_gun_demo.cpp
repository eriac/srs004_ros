#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <string>
#include "math.h"

ros::Publisher cmd_vel_pub;
ros::Publisher cmd_cmd_pub;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	geometry_msgs::Twist gun_vel;
	gun_vel.angular.y=joy_msg.axes[1];
	gun_vel.angular.z=joy_msg.axes[0];
	cmd_vel_pub.publish(gun_vel);

	std_msgs::String command;
	if(joy_msg.buttons[0]){
		command.data="shot";
		cmd_cmd_pub.publish(command);
	}
	else if(joy_msg.buttons[1]){
		command.data="laser_on";
		cmd_cmd_pub.publish(command);
	}
	else if(joy_msg.buttons[2]){
		command.data="laser_off";
		cmd_cmd_pub.publish(command);
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
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("gun_twist", 1000);
    cmd_cmd_pub = n.advertise<std_msgs::String>("gun_command", 1000);
	
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
