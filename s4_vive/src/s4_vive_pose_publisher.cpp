#include "ros/ros.h"
  
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <s4_vive/vive_general.h>


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
geometry_msgs::Pose get_goal(){
	static tf::TransformListener tflistener;
	geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id=goal_frame;
	source_pose.pose.orientation.w=1.0;
	geometry_msgs::PoseStamped target_pose;
	try{
		tflistener.waitForTransform("world", goal_frame, ros::Time(0), ros::Duration(1.0));
		tflistener.transformPose("world",ros::Time(0),source_pose, goal_frame, target_pose);
		//printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
	}
	catch(...){
		printf("error\n");
	}
	return target_pose.pose;
}
ros::Publisher pose_pub;
void vive_callback(const s4_vive::vive_general& vive_msg){
	if(vive_msg.buttons[0]){
		geometry_msgs::Pose goal_pose=get_goal();
		pose_pub.publish(goal_pose);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_vive_body_tf");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("goal_frame", goal_frame);

	//publisher
	pose_pub   = n.advertise<geometry_msgs::Pose>("pose", 1);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("data", 10, vive_callback);

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

