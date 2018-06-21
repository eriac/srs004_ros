#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"

#include "math.h"
#include <sstream>
#include <string>

ros::Publisher marker_pub;
std::string laser_link_name="";
void twist_callback(const geometry_msgs::Twist& twist_msg){
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.resize(2);

	//marker0
	float length=sqrt(pow(twist_msg.linear.x,2)+pow(twist_msg.linear.y,2));
	float angle =atan2(twist_msg.linear.y, twist_msg.linear.x);

	marker_array.markers[0].header.frame_id = laser_link_name;
	marker_array.markers[0].header.stamp = ros::Time::now();
	marker_array.markers[0].ns = "basic_shapes";
	marker_array.markers[0].id = 0;
	marker_array.markers[0].lifetime = ros::Duration();
	
	marker_array.markers[0].type = visualization_msgs::Marker::ARROW;
	if(length>0.01){
		marker_array.markers[0].action = visualization_msgs::Marker::ADD;
	}
	else{
		marker_array.markers[0].action = visualization_msgs::Marker::DELETE;		
	}
	marker_array.markers[0].scale.x = 0.02;
	marker_array.markers[0].scale.y = 0.04;
	marker_array.markers[0].scale.z = 0.1;

	marker_array.markers[0].points.resize(2);
	marker_array.markers[0].points[0].x=0.2*cos(angle);
	marker_array.markers[0].points[0].y=0.2*sin(angle);
	marker_array.markers[0].points[0].z=0;
	marker_array.markers[0].points[1].x=(0.2+length)*cos(angle);
	marker_array.markers[0].points[1].y=(0.2+length)*sin(angle);
	marker_array.markers[0].points[1].z=0;

	marker_array.markers[0].color.r = 0.0f;
	marker_array.markers[0].color.g = 1.0f;
	marker_array.markers[0].color.b = 0.0f;
	marker_array.markers[0].color.a = 1.0f;

	//marker1
	marker_array.markers[1].header.frame_id = laser_link_name;
	marker_array.markers[1].header.stamp = ros::Time::now();
	marker_array.markers[1].ns = "basic_shapes";
	marker_array.markers[1].id = 1;
	marker_array.markers[1].lifetime = ros::Duration();

	marker_array.markers[1].type = visualization_msgs::Marker::LINE_STRIP;
	if(length>0.01){
		marker_array.markers[0].action = visualization_msgs::Marker::ADD;
	}
	else{
		marker_array.markers[0].action = visualization_msgs::Marker::DELETE;		
	}
	marker_array.markers[1].scale.x = 0.01;

	float step=0.1;
	int line_num=1;
	if(twist_msg.angular.z>0){
		line_num=(twist_msg.angular.z/step)+1;
		marker_array.markers[1].points.resize(line_num+1);
		for(int i=0;i<(line_num);i++){
			marker_array.markers[1].points[i].x=0.3*cos(i*step);
			marker_array.markers[1].points[i].y=0.3*sin(i*step);
			marker_array.markers[1].points[i].z=0;
		}
	}
	else{
		line_num=(-twist_msg.angular.z/step)+1;
		marker_array.markers[1].points.resize(line_num+1);
		for(int i=0;i<(line_num);i++){
			marker_array.markers[1].points[i].x=0.3*cos(-i*step);
			marker_array.markers[1].points[i].y=0.3*sin(-i*step);
			marker_array.markers[1].points[i].z=0;
		}
	}
	marker_array.markers[1].points[line_num].x=0.3*cos(twist_msg.angular.z);
	marker_array.markers[1].points[line_num].y=0.3*sin(twist_msg.angular.z);
	marker_array.markers[1].points[line_num].z=0;

	marker_array.markers[1].color.r = 1.0f;
	marker_array.markers[1].color.g = 0.0f;
	marker_array.markers[1].color.b = 0.0f;
	marker_array.markers[1].color.a = 1.0f;

	marker_pub.publish(marker_array);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_omni_twist_marker_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("twist_link_name", laser_link_name);

	//publish
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

	//Subscribe
	ros::Subscriber joy_sub     = n.subscribe("cmd_vel", 10, twist_callback);
		
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
