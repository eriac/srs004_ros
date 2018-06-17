#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"

#include "math.h"
#include <sstream>
#include <string>


int   gun_remain=0;
void remain_callback(const std_msgs::Float32& float_msg){
	gun_remain=float_msg.data;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_shot_marker_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string gun_link_name="";
	pn.getParam("gun_link_name", gun_link_name);

	//publish
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 1);

	//Subscribe
	ros::Subscriber remain_sub  = n.subscribe("remain", 10, remain_callback);
			
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		//publish shot
		if(gun_remain>0){
			static int timer=0;
			if(timer%2==0){
				gun_remain--;
			}
			timer++;
		}
		
		//shot marker
		visualization_msgs::Marker gun_marker;
		gun_marker.header.frame_id = gun_link_name;
		gun_marker.header.stamp = ros::Time::now();
		gun_marker.ns = "basic_shapes";
		gun_marker.id = 0;
		gun_marker.lifetime = ros::Duration(0.5);
		gun_marker.frame_locked =true;
		
		gun_marker.type = visualization_msgs::Marker::ARROW;
		//gun_marker.action = visualization_msgs::Marker::ADD;
		
		gun_marker.scale.x = 0.01;
		gun_marker.scale.y = 0.02;
		gun_marker.scale.z = 0.05;

		gun_marker.points.resize(2);
		gun_marker.points[0].x=0.2;
		gun_marker.points[0].y=0;
		gun_marker.points[0].z=0;
		//gun_marker.points[1].x=0.2+0.3;
		gun_marker.points[1].y=0;
		gun_marker.points[1].z=0;

		gun_marker.color.r = 1.0f;
		gun_marker.color.g = 0.0f;
		gun_marker.color.b = 0.0f;
		gun_marker.color.a = 1.0f;
		
		if(gun_remain>5){
			gun_marker.action = visualization_msgs::Marker::ADD;
			gun_marker.points[1].x=0.2+0.1*5;
			marker_pub.publish(gun_marker);
		}
		else if(gun_remain>0){
			gun_marker.action = visualization_msgs::Marker::ADD;
			gun_marker.points[1].x=0.2+0.1*gun_remain;
			marker_pub.publish(gun_marker);
		}

		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
