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
float linear_th1    = 0.01;
float linear_th2    = 0.30;
float angular_speed = 2.0;
float angular_th1   = 0.02;
float angular_th2   = 1.00;

float GetRPY(geometry_msgs::Quaternion q){
	double roll, pitch, yaw=0;
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	return (float)yaw;
}

std::string target_frame="";
geometry_msgs::Pose get_relative(geometry_msgs::Pose target){
	static tf::TransformListener tflistener;
	geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id="world";
	source_pose.pose=target;
	geometry_msgs::PoseStamped target_pose;
	tflistener.waitForTransform(target_frame, "world", ros::Time(0), ros::Duration(1.0));
	tflistener.transformPose(target_frame,ros::Time(0),source_pose,"world",target_pose);
	return target_pose.pose;
}

bool move_enable;
geometry_msgs::Pose move_pose;
int published_timer=0;
void pose_callback(const geometry_msgs::Pose& pose_msg){
	move_pose=pose_msg;
	move_enable=true;
	//geometry_msgs::Pose tmp=get_relative(pose_msg);
	geometry_msgs::Pose tmp=pose_msg;
	printf("x:%f, y:%f\n",tmp.position.x,tmp.position.y);
	published_timer=0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_operate_pose_move");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("target_frame", target_frame);

	bool slow_start=false;
	pn.getParam("slow_start", slow_start);
	geometry_msgs::Twist cmd_vel_last;

	//publisher
	ros::Publisher move_twist_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
 
	//subscriber
	ros::Subscriber pose_sub   = n.subscribe("pose", 10, pose_callback); 

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		if(move_enable){
			geometry_msgs::Pose relative_pose=get_relative(move_pose);
			float pos_yaw=GetRPY(relative_pose.orientation);
			printf("yaw:%f\n",pos_yaw);
	
			geometry_msgs::Twist command;
			float pos_length=sqrt(pow(relative_pose.position.x,2)+pow(relative_pose.position.y,2));
			if(pos_length<linear_th1 && fabs(pos_yaw)<angular_th1){
				move_enable=false;
				printf("set false %f\n",pos_length);
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

				if(fabs(pos_yaw)<angular_th2){
					command.angular.z=(angular_speed/angular_th2)*pos_yaw;
				}
				else{
					command.angular.z=angular_speed*(pos_yaw/fabs(pos_yaw));
					ROS_INFO("roll");
				}
			}
			if(slow_start){//slow
				float diff_last=sqrt(cmd_vel_last.linear.x*cmd_vel_last.linear.x+cmd_vel_last.linear.y*cmd_vel_last.linear.y);
				float diff_command=sqrt(cmd_vel_last.linear.x*cmd_vel_last.linear.x+cmd_vel_last.linear.y*cmd_vel_last.linear.y);

				float diff_x=command.linear.x-cmd_vel_last.linear.x;
				float diff_y=command.linear.y-cmd_vel_last.linear.y;
				float diff_s=sqrt(diff_x*diff_x+diff_y*diff_y);
				
				float alpha=1.0;
				if(published_timer<7)alpha=0.2;
				else if(published_timer<12)alpha=0.9;
				else alpha=1.0;
				published_timer++;

				command.linear.x=alpha*command.linear.x+(1-alpha)*cmd_vel_last.linear.x;
				command.linear.y=alpha*command.linear.y+(1-alpha)*cmd_vel_last.linear.y;
				move_twist_pub.publish(command);
				cmd_vel_last=command;
			}
			else{
				move_twist_pub.publish(command);
			}
		}
		else{//!move_enable
			cmd_vel_last.linear.x=0.0;
			cmd_vel_last.linear.y=0.0;
			cmd_vel_last.linear.z=0.0;
			cmd_vel_last.angular.x=0.0;
			cmd_vel_last.angular.y=0.0;
			cmd_vel_last.angular.z=0.0;
		}

	
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

