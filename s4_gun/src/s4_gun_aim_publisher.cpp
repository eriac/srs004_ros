#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include <string>


enum Eaim_mode { AIM_NONE, AIM_VEL,AIM_POS,AIM_TARGET };
Eaim_mode aim_mode = AIM_NONE;

float range_filter(float value, float lower, float upper){
	float output=0.0;
	if(value<lower)output=lower;
	else if(value>upper)output=upper;
	else output=value;
	return output;
}

geometry_msgs::Twist vel_last;
void vel_callback(const geometry_msgs::Twist& twist_msg){
	vel_last=twist_msg;
	aim_mode=AIM_VEL;
}

geometry_msgs::Pose pos_last;
void pos_callback(const geometry_msgs::Pose& pose_msg){
	pos_last=pose_msg;
	aim_mode=AIM_POS;
}

geometry_msgs::Point target_last;
void target_callback(const geometry_msgs::Point& point_msg){
	target_last=point_msg;
	aim_mode=AIM_TARGET;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_aim_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher command_pub = n.advertise<geometry_msgs::Pose>("aim_command", 10);
	ros::Publisher joint_pub   = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	
	//Subscribe
	ros::Subscriber vel_sub = n.subscribe("aim_vel", 10, vel_callback);
	ros::Subscriber pos_sub = n.subscribe("aim_pos", 10, pos_callback);
	ros::Subscriber target_sub = n.subscribe("aim_target", 10, target_callback);

	//rosparam
	std::string joint1_name="";
	std::string joint2_name="";
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
		static geometry_msgs::Pose current_pose;

		if(aim_mode==AIM_VEL){
			float temp1=current_pose.orientation.z+vel_last.angular.z*dt;
			if(yaw_lower_limit<=temp1 && temp1<=yaw_upper_limit){
				current_pose.orientation.z=temp1;
			}
			float temp2=current_pose.orientation.y+vel_last.angular.y*dt;
			if(pitch_lower_limit<=temp2 && temp2<=pitch_upper_limit){
				current_pose.orientation.y=temp2;
			}
			command_pub.publish(current_pose);
		}
		else if(aim_mode==AIM_POS){
			current_pose.orientation.z=range_filter(pos_last.orientation.z,   yaw_lower_limit, yaw_upper_limit);
			current_pose.orientation.y=range_filter(pos_last.orientation.y, pitch_lower_limit, pitch_upper_limit);
			command_pub.publish(current_pose);
		}

		//publish jointstates
		sensor_msgs::JointState js0;
		js0.header.stamp = ros::Time::now();
		js0.name.resize(2);
		js0.name[0]=joint1_name;
		js0.name[1]=joint2_name;
		js0.position.resize(2);
		js0.position[0]=current_pose.orientation.z;
		js0.position[1]=current_pose.orientation.y;
		joint_pub.publish(js0);

		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

