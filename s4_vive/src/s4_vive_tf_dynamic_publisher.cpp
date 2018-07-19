#include "ros/ros.h"
  
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include <string>
#include <iostream>
#include <sstream>

std::string world_frame="world";
std::string from_frame="";
std::string to_frame="";

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_vive_tf_overlapper");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("world_frame", world_frame);
	pn.getParam("from_frame",  from_frame);
	pn.getParam("to_frame",    to_frame);

	tf::TransformListener tflistener;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		geometry_msgs::PoseStamped source_pose;
		source_pose.header.frame_id=from_frame;
		source_pose.pose.orientation.w=1.0;
		geometry_msgs::PoseStamped target_pose;
		try{
			tflistener.waitForTransform(world_frame, from_frame, ros::Time(0), ros::Duration(1.0));
			tflistener.transformPose(world_frame,ros::Time(0),source_pose, from_frame, target_pose);
		
			//printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
			
			float x = target_pose.pose.position.x;
			float y = target_pose.pose.position.y;
			float z = target_pose.pose.position.z;
			
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(
				target_pose.pose.position.x,
				target_pose.pose.position.y,
				target_pose.pose.position.z));
			transform.setRotation( tf::Quaternion(
				target_pose.pose.orientation.x,
				target_pose.pose.orientation.y,
				target_pose.pose.orientation.z,
				target_pose.pose.orientation.w));

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, to_frame));

		}
		catch(...){
			printf("tracking tf error\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

