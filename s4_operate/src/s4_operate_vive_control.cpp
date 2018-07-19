#include "ros/ros.h"
  
#include "math.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "jsk_rviz_plugins/OverlayMenu.h"



#include <string>
#include <iostream>
#include <sstream>

void set_frame(std::string name, float *pos, float *dir){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(  tf::Vector3(   pos[0], pos[1], pos[2]) );
	transform.setRotation(tf::Quaternion(dir[0], dir[1], dir[2], dir[3]));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));	
}
std::string goal_frame="";
geometry_msgs::Pose get_goal(float pos_z){
	static tf::TransformListener tflistener;
	geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id=goal_frame;
	source_pose.pose.position.z=pos_z;
	source_pose.pose.orientation.w=1.0;
	geometry_msgs::PoseStamped target_pose;
	try{
		tflistener.waitForTransform("world", goal_frame, ros::Time(0), ros::Duration(1.0));
		tflistener.transformPose("world",ros::Time(0),source_pose, goal_frame, target_pose);
		//printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
	}
	catch(...){
		printf("error(get gloal frame)\n");
	}
	return target_pose.pose;
}
std::string pointer_frame="";
float get_pointer(){
	static tf::TransformListener tflistener;
	geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id=pointer_frame;
	source_pose.pose.orientation.w=1.0;
	geometry_msgs::PoseStamped target_pose;
	try{
		tflistener.waitForTransform("world", pointer_frame, ros::Time(0), ros::Duration(1.0));
		tflistener.transformPose("world",ros::Time(0),source_pose, pointer_frame, target_pose);
		//printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
	}
	catch(...){
		printf("error(get pointer frame)\n");
	}
	double r,p,y;
	tf::Quaternion quat(
		target_pose.pose.orientation.x,
		target_pose.pose.orientation.y,
		target_pose.pose.orientation.z,
		target_pose.pose.orientation.w);
	tf::Matrix3x3(quat).getRPY(r, p, y);
	return r;
}

ros::Time arm_time=ros::Time(0);
void arm_callback(const std_msgs::Empty& empty_msg){
	arm_time=ros::Time::now();
}

ros::Publisher pose_pub;
ros::Publisher point_pub;
ros::Publisher laser_pub;
ros::Publisher shot_pub;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	//robot pose
	bool enable_pose=false;
	static bool initial_pose=true;
	static geometry_msgs::Pose goal_pose;
	if(initial_pose){
		initial_pose=false;
		goal_pose.orientation.w=1.0;
	}

	//point
	if(joy_msg.buttons[0]){//triger
		enable_pose=true;
		goal_pose.position=get_goal(0).position;
	}
	//yaw
	static float now_robot_yaw=0.0;
	static float last_robot_yaw=0.0;
	static float last_pointer_roll=0.0;
	if(joy_msg.buttons[3]){//grip
		enable_pose=true;
		now_robot_yaw=last_robot_yaw+(-0.5)*(get_pointer()-last_pointer_roll);

		tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, now_robot_yaw);
		geometry_msgs::Quaternion robot_orientation;
		quaternionTFToMsg(quat,robot_orientation);
		goal_pose.orientation=robot_orientation;
	}
	else{
		last_pointer_roll=get_pointer();
		last_robot_yaw=now_robot_yaw;
	}

	//publish pose
	if(enable_pose){
		pose_pub.publish(goal_pose);
	}

	//publish gun target
	static std::string gun_state="standby";
	static ros::Time pre_time;
	if(gun_state=="standby"){
		if(joy_msg.buttons[1]==1){
			if(ros::Time::now() < arm_time+ros::Duration(30.0)){
				gun_state="pre";
				pre_time=ros::Time::now();
			}
			else{
				gun_state="inhibit";
				pre_time=ros::Time::now();
			}
		}
	}
	else if(gun_state=="pre"){
		geometry_msgs::Pose goal_pose=get_goal(0.1);
		geometry_msgs::PointStamped target_point;
		target_point.header.frame_id="world";
		target_point.point=goal_pose.position;
		point_pub.publish(target_point);

		int n_count=(ros::Time::now()-pre_time).nsec/10000000;
		std_msgs::Bool laser;
		laser.data=false;
		if((n_count%25)<12)laser.data=true;
		laser_pub.publish(laser);

		if(ros::Time::now() > pre_time+ros::Duration(1.0)){
			gun_state="ready";
		}
		else if(joy_msg.buttons[1]==0){
			gun_state="post";
		}
		else if(joy_msg.buttons[1]==2){
			gun_state="post";
		}
	}
	else if(gun_state=="ready"){
		geometry_msgs::Pose goal_pose=get_goal(0.1);
		geometry_msgs::PointStamped target_point;
		target_point.header.frame_id="world";
		target_point.point=goal_pose.position;
		point_pub.publish(target_point);

		std_msgs::Bool laser;
		laser.data=true;
		laser_pub.publish(laser);

		if(joy_msg.buttons[1]==0){
			gun_state="post";
		}
		else if(joy_msg.buttons[1]==2){
			std_msgs::Int32 shot_msg;
			shot_msg.data=3;
			shot_pub.publish(shot_msg);

			gun_state="post";
		}
	}
	else if(gun_state=="inhibit"){
		geometry_msgs::Pose goal_pose=get_goal(0.1);
		geometry_msgs::PointStamped target_point;
		target_point.header.frame_id="world";
		target_point.point=goal_pose.position;
		point_pub.publish(target_point);

		int n_count=(ros::Time::now()-pre_time).nsec/10000000;
		std_msgs::Bool laser;
		laser.data=false;
		if((n_count%25)<12)laser.data=true;
		laser_pub.publish(laser);

		if(joy_msg.buttons[1]==0){
			gun_state="post";
		}
	}
	else if(gun_state=="post"){
		std_msgs::Bool laser;
		laser.data=false;
		laser_pub.publish(laser);

		if(joy_msg.buttons[1]==0){
			gun_state="standby";
		}
	}
	ROS_INFO("%s",gun_state.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_vive_body_tf");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("goal_frame", goal_frame);
	pn.getParam("pointer_frame", pointer_frame);

	//publisher
	pose_pub   = n.advertise<geometry_msgs::Pose>("pose", 1);
	point_pub  = n.advertise<geometry_msgs::PointStamped>("target", 1);	
	laser_pub  = n.advertise<std_msgs::Bool>("laser", 1);	
	shot_pub   = n.advertise<std_msgs::Int32>("shot", 1);	
	ros::Publisher arm_pub   = n.advertise<jsk_rviz_plugins::OverlayMenu>("arm_info", 1);	

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);
	ros::Subscriber arm_sub   = n.subscribe("arm", 10, arm_callback);

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		jsk_rviz_plugins::OverlayMenu arm_info;
		arm_info.action=jsk_rviz_plugins::OverlayMenu::ACTION_SELECT;
		if(ros::Time::now() < arm_time+ros::Duration(30.0)){
			arm_info.current_index=0;
		}
		else arm_info.current_index=1;
		arm_info.menus.resize(2);
		arm_info.menus[0]="Arm";
		arm_info.menus[1]="Disarm";
		arm_info.title="Gun Master";
		arm_pub.publish(arm_info);
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

