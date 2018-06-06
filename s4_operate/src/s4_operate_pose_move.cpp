#include "ros/ros.h"
  
#include "math.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include <string>
#include <iostream>
#include <sstream>

#include "tf/transform_listener.h"

float linear_speed  = 0.5;
float linear_th1    = 0.02;
float linear_th2    = 0.30;
float angular_speed = 1.5;
float angular_th1   = 0.02;
float angular_th2   = 0.40;

float GetRPY(geometry_msgs::Quaternion q){
	double roll, pitch, yaw=0;
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	return (float)yaw;
}

geometry_msgs::Pose get_relative(geometry_msgs::Pose target){
	static tf::TransformListener tflistener;
	geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id="world";
	source_pose.pose=target;
	geometry_msgs::PoseStamped target_pose;
	tflistener.waitForTransform("robot0/base_link", "world", ros::Time(0), ros::Duration(1.0));
	tflistener.transformPose("robot0/base_link",ros::Time(0),source_pose,"world",target_pose);
	return target_pose.pose;
}

bool move_enable;
geometry_msgs::Pose move_pose;
void pose_callback(const geometry_msgs::Pose& pose_msg){
	move_pose=pose_msg;
	move_enable=true;
	//geometry_msgs::Pose tmp=get_relative(pose_msg);
	geometry_msgs::Pose tmp=pose_msg;
	printf("x:%f, y:%f\n",tmp.position.x,tmp.position.y);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_operate_pose_move");
	ros::NodeHandle n;
	
	//publisher
	ros::Publisher move_twist_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
 
	//subscriber
	ros::Subscriber pose_sub   = n.subscribe("pose", 10, pose_callback); 

	ros::Rate loop_rate(10); 
	while (ros::ok()){
		if(move_enable){
			geometry_msgs::Pose relative_pose=get_relative(move_pose);
			float pos_yaw=GetRPY(relative_pose.orientation);
			printf("yaw:%f\n",pos_yaw);
	
			geometry_msgs::Twist command;
			float pos_length=sqrt(pow(relative_pose.position.x,2)+pow(relative_pose.position.y,2));
			if(pos_length<linear_th1 && fabs(pos_yaw)<0.02){
				move_enable=false;
				printf("set false\n");
			}
			else{
				if(pos_length<linear_th2){
					float linear_k=(linear_speed/linear_th2);
					command.linear.x=linear_k * relative_pose.position.x;
					command.linear.y=linear_k * relative_pose.position.y;
					printf("set mid x:%f, y:%f\n",	command.linear.x,	command.linear.y);
				}
				else{
					command.linear.x=linear_speed*relative_pose.position.x/pos_length;
					command.linear.y=linear_speed*relative_pose.position.y/pos_length;
					printf("set far x:%f, y:%f\n",	command.linear.x,	command.linear.y);
				}

				if(fabs(pos_yaw)<0.5){
					command.angular.z=2.0*pos_yaw;
				}
				else{
					command.angular.z=1.0*(pos_yaw/fabs(pos_yaw));
				}
			}
			move_twist_pub.publish(command);
		}

	
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

