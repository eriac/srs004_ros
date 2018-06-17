#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"

#include "math.h"
#include <sstream>
#include <string>

ros::Publisher marker_pub;
std::string laser_link_name="";
void laser_callback(const std_msgs::Bool& bool_msg){
	visualization_msgs::Marker laser_marker;
	laser_marker.header.frame_id = laser_link_name;
	laser_marker.header.stamp = ros::Time::now();
	laser_marker.ns = "basic_shapes";
	laser_marker.id = 1;
	laser_marker.lifetime = ros::Duration();
	laser_marker.frame_locked =true;
	
	laser_marker.type = visualization_msgs::Marker::ARROW;
	//laser_marker.action = visualization_msgs::Marker::ADD;
	
	laser_marker.scale.x = 0.005;
	laser_marker.scale.y = 0.01;
	laser_marker.scale.z = 0.05;

	laser_marker.points.resize(2);
	laser_marker.points[0].x=0;
	laser_marker.points[0].y=0;
	laser_marker.points[0].z=0;
	laser_marker.points[1].x=1.5;
	laser_marker.points[1].y=0;
	laser_marker.points[1].z=0;

	laser_marker.color.r = 0.0f;
	laser_marker.color.g = 1.0f;
	laser_marker.color.b = 0.0f;
	laser_marker.color.a = 1.0f;
	
	if(bool_msg.data){
		laser_marker.action = visualization_msgs::Marker::ADD;
		marker_pub.publish(laser_marker);
	}
	else{
		laser_marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(laser_marker);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_gun_laser_marker_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("laser_link_name", laser_link_name);

	//publish
	//ros::Publisher 
	marker_pub = n.advertise<visualization_msgs::Marker>("marker", 1);

	//Subscribe
	ros::Subscriber laser_sub  = n.subscribe("laser", 10, laser_callback);
		
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
