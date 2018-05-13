#include "ros/ros.h"
  
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include <sstream>
#include <string>

std::string joint1_name="";
std::string joint2_name="";

ros::Publisher laser_pub;
geometry_msgs::Twist twist_last;
bool twist_enable;
void twist_callback(const geometry_msgs::Twist& twist_msg){
	twist_last=twist_msg;
	twist_enable=true;
}
bool shot_flag=false;
void command_callback(const std_msgs::String& command_msg){
	if(command_msg.data=="shot")shot_flag=true;
	else if(command_msg.data=="laser_on"){
		std_msgs::Int32 int_data;
		int_data.data=1;
		laser_pub.publish(int_data);
	}
	else if(command_msg.data=="laser_off"){
		std_msgs::Int32 int_data;
		int_data.data=0;
		laser_pub.publish(int_data);
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_gun_commander");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("twist_out", 1000);
	ros::Publisher shot_pub  = n.advertise<std_msgs::Int32>("shot", 1000);
	laser_pub = n.advertise<std_msgs::Int32>("laser", 1000);
	
	//Subscribe
	ros::Subscriber twist_sub   = n.subscribe("twist_in", 10, twist_callback);
	ros::Subscriber command_sub = n.subscribe("command", 10, command_callback);

	//rosparam
	pn.getParam("joint1_name", joint1_name);
	pn.getParam("joint2_name", joint2_name);

	float yaw_velocity=1.0;
	float pitch_velocity=1.0;
	float yaw_lower_limit=-1.0;
	float yaw_upper_limit=1.0;
	float pitch_lower_limit=-1.0;
	float pitch_upper_limit=1.0;

	pn.getParam("yaw_lower_limit", yaw_lower_limit);
	pn.getParam("yaw_upper_limit", yaw_upper_limit  );
	pn.getParam("pitch_lower_limit", pitch_lower_limit);
	pn.getParam("pitch_upper_limit", pitch_upper_limit  );
	pn.getParam("yaw_velocity",   yaw_velocity);
	pn.getParam("pitch_velocity", pitch_velocity);
		
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		static float yaw=0.0;
		static float pitch=0.0;
		if(twist_enable){
			float temp1=yaw+yaw_velocity*twist_last.angular.z*dt;
			if(yaw_lower_limit<=temp1 && temp1<=yaw_upper_limit){
				yaw=temp1;
			}
			float temp2=pitch+pitch_velocity*twist_last.angular.y*dt;
			if(pitch_lower_limit<=temp2 && temp2<=pitch_upper_limit){
				pitch=temp2;
			}
			geometry_msgs::Twist twist_msg;
			twist_msg.angular.z=yaw;
			twist_msg.angular.y=pitch;
			twist_pub.publish(twist_msg);
			
			if(shot_flag){
				std_msgs::Int32 float_data;
				float_data.data=1;
				shot_pub.publish(float_data);
				shot_flag=false;
			}
		}
		
		//publish jointstates
		sensor_msgs::JointState js0;
		js0.header.stamp = ros::Time::now();
		js0.name.resize(2);
		js0.name[0]=joint1_name;
		js0.name[1]=joint2_name;
		js0.position.resize(2);
		js0.position[0]=yaw;
		js0.position[1]=pitch;
		joint_pub.publish(js0);


		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

