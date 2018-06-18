#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <tf/tf.h>
#include "tf/transform_broadcaster.h"

#include <string>
#include "math.h"

geometry_msgs::Twist odm_now;
void odm_callback(const geometry_msgs::Twist& odm_msg){
    odm_now=odm_msg;
}

void set_robot(float *position, float *direction, std::string base_link, std::string world_link){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
	tf::Quaternion q;
	q.setRPY(direction[0], direction[1], direction[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_link, base_link));	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_operate_position");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string world_link="";
	std::string base_link="";
	pn.getParam("world_link", world_link);
	pn.getParam("base_link",  base_link);

	float pos_x=0.0;
	float pos_y=0.0;
	float rot_z=0.0;
	pn.getParam("pos_x", pos_x);
	pn.getParam("pos_y", pos_y);
	pn.getParam("rot_z", rot_z);

    //subscriibe
	ros::Subscriber odm_sub   = n.subscribe("odm_vel", 10, odm_callback);

	float dt=1.0/20;
	ros::Rate loop_rate(20);
	float position[3]={pos_x, pos_y, 0};
	float direction[3]={0, 0, rot_z};
	while (ros::ok()){
		position[0]+=(cos(direction[2])*odm_now.linear.x-sin(direction[2])*odm_now.linear.y)*dt;
		position[1]+=(sin(direction[2])*odm_now.linear.x+cos(direction[2])*odm_now.linear.y)*dt;
		position[2]=0.0;//0.019
		direction[0]=0.0;
		direction[1]=0.0;
		direction[2]+=odm_now.angular.z*dt;
		set_robot(position, direction, base_link, world_link);
        ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
