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

std::string pointer_frame="pointer_link";
std::string goal_frame="goal_link";

void set_goal(float x, float y, float z){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x,y,z) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", goal_frame));

}
void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw){
	//bulletのクオータニオンに変換
	tf::Quaternion btq(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);
}
ros::Publisher marker_pub;
void draw_line(float *source, float *target){
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
    	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.scale.x = 0.02;
	marker.scale.y = 0.04;
	marker.scale.z = 0.04;
	
	marker.points.resize(2);
	marker.points[0].x=source[0];
	marker.points[0].y=source[1];
	marker.points[0].z=source[2];
	marker.points[1].x=target[0];
	marker.points[1].y=target[1];
	marker.points[1].z=target[2];

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);


}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_vive_goal_publisher");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("pointer_frame", pointer_frame);
    printf("%s\n",pointer_frame.c_str());
	pn.getParam("goal_frame",   goal_frame);
	tf::TransformListener tflistener;

    //publisher
    marker_pub   = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


	ros::Rate loop_rate(20); 
	while (ros::ok()){

		geometry_msgs::PoseStamped source_pose;
		source_pose.header.frame_id=pointer_frame;
		source_pose.pose.orientation.w=1.0;
		geometry_msgs::PoseStamped target_pose;
		try{
			tflistener.waitForTransform("world", pointer_frame, ros::Time(0), ros::Duration(1.0));
			tflistener.transformPose("world",ros::Time(0),source_pose, pointer_frame, target_pose);
		
			//printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
			double roll,pitch,yaw;
			GetRPY(target_pose.pose.orientation,roll,pitch,yaw);
			//printf("O R:%f, P:%f, Y:%f\n",roll,pitch,yaw);
		
			if(pitch>0.2){
				float l1=target_pose.pose.position.z/tan(pitch);
				float x =target_pose.pose.position.x+l1*cos(yaw);
				float y =target_pose.pose.position.y+l1*sin(yaw);
				set_goal(x,y,0);
				float source[3]={target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z};
				float target[3]={x,y,0};
				draw_line(source,target);
			}
		}
		catch(...){
			printf("error\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

