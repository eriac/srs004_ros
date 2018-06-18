#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <tf/tf.h>
#include "tf/transform_broadcaster.h"

#include <string>
#include "math.h"

ros::Publisher cmd_pub;
std::string robot="";
float linear_velocity=1.0;
float angular_velocity=1.0;

float target[2]={1.0, 0.0};
float move[3]={0.0,0.0,0.0}; 
void joy_callback(const sensor_msgs::Joy& joy_msg){
	if(joy_msg.buttons[4]){
		target[0]=1.0;
		target[1]=0.0;
	}
	else if(joy_msg.buttons[5]){
		target[0]=0.0;
		target[1]=1.0;
	}
	else if(joy_msg.buttons[6]){
		target[0]=-1.0;
		target[1]=0.0;
	}
	else if(joy_msg.buttons[7]){
		target[0]=0.0;
		target[1]=-1.0;
	}

	if(joy_msg.buttons[0]){
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x =move[0];
		cmd_vel.linear.y =move[1];
		cmd_vel.angular.z=move[2];
		cmd_pub.publish(cmd_vel);
	}
	else{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x =linear_velocity*joy_msg.axes[1];
		cmd_vel.linear.y =linear_velocity*joy_msg.axes[0];
		cmd_vel.angular.z=angular_velocity*joy_msg.axes[2];
		cmd_pub.publish(cmd_vel);
	}
}

geometry_msgs::Twist odm_now;
void odm_callback(const geometry_msgs::Twist& odm_msg){
    odm_now=odm_msg;
}

void set_robot(float *position, float *direction){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
	tf::Quaternion q;
	q.setRPY(direction[0], direction[1], direction[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot+"/base_link"));	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "s4_omni_demo");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	//rosparam
	pn.getParam("robot", robot);
	pn.getParam("linear_velocity",  linear_velocity);
	pn.getParam("angular_velocity", angular_velocity);

    //publish
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
    //subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);
	ros::Subscriber odm_sub   = n.subscribe("odm_vel", 10, odm_callback);

	float dt=1.0/20;
	ros::Rate loop_rate(20);

	while (ros::ok()){
		static float position[3]={0};
		static float direction[3]={0};
		position[0]+=(cos(direction[2])*odm_now.linear.x-sin(direction[2])*odm_now.linear.y)*dt;
		position[1]+=(sin(direction[2])*odm_now.linear.x+cos(direction[2])*odm_now.linear.y)*dt;
		position[2]=0.0;
		direction[0]=0.0;
		direction[1]=0.0;
		direction[2]+=odm_now.angular.z*dt;
		set_robot(position,direction);	
		
		float diff[2]={0.0};
		diff[0]=target[0]-position[0];
		diff[1]=target[1]-position[1];
		
		move[0]=(+cos(direction[2])*diff[0]+sin(direction[2])*diff[1])*0.5;
		move[1]=(-sin(direction[2])*diff[0]+cos(direction[2])*diff[1])*0.5;
		move[2]=0.0;

		ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
