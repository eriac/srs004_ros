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

std::string body_frame="";
void vive_callback(const s4_vive::vive_general& vive_msg){
	float pos[3]={0.0,0.0,0.0};
	float dir[4]={0.0,0.0,0.0,1.0};
	pos[0]=vive_msg.position.x;
	pos[1]=vive_msg.position.y;
	pos[2]=vive_msg.position.z;
	dir[0]=vive_msg.orientation.x;
	dir[1]=vive_msg.orientation.y;
	dir[2]=vive_msg.orientation.z;
	dir[3]=vive_msg.orientation.w;
	set_frame(body_frame, pos, dir);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_vive_body_tf");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("body_frame", body_frame);


	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("data", 10, vive_callback);

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

