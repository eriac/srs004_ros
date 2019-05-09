#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <sstream>
#include <string>

class omni_odom{
};

float wheel_base=0.100;
float wheel_radius=0.20;
float publish_rate=20;
std::string frame_id="odom";
std::vector<double> pose_covariance_;
std::vector<double> twist_covariance_;

geometry_msgs::Twist wheel_invert(float *in){
	geometry_msgs::Twist out;
	float lv=1.0/wheel_radius;
	float av=wheel_base/wheel_radius;
	float r3=sqrt(3);
	out.linear.x  = +(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2];
	out.linear.y  = -(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2];
	out.angular.z = -(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2];
	return out;
}
geometry_msgs::Pose update_pose(geometry_msgs::Pose last_pose, geometry_msgs::Twist speed, float dt){
	geometry_msgs::Pose next_pose;

	tf::Quaternion quat_tmp;
    double roll, pitch, last_yaw=0;
    //get yaw
	quaternionMsgToTF(last_pose.orientation, quat_tmp);
	tf::Matrix3x3(quat_tmp).getRPY(roll, pitch, last_yaw); 
	//update
	next_pose.position.x = last_pose.position.x + (cos(last_yaw+speed.angular.z * dt/2) * speed.linear.x - sin(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
	next_pose.position.y = last_pose.position.y + (sin(last_yaw+speed.angular.z * dt/2) * speed.linear.x + cos(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
	float next_yaw = last_yaw + speed.angular.z * dt;
	//sey yaw
	quat_tmp=tf::createQuaternionFromRPY(0,0,next_yaw);
    quaternionTFToMsg(quat_tmp, next_pose.orientation);
	return next_pose;
}

geometry_msgs::Pose body_position;
ros::Publisher odom_pub;
	
typedef message_filters::sync_policies::ApproximateTime<control_msgs::JointControllerState, control_msgs::JointControllerState, control_msgs::JointControllerState> StateSyncPolicy;
void sync_callback(const control_msgs::JointControllerState &state0_msg,
                   const control_msgs::JointControllerState &state1_msg,
                   const control_msgs::JointControllerState &state2_msg) {

  float wheel_speed[3]={(float)state0_msg.process_value, (float)state1_msg.process_value, (float)state2_msg.process_value};
  geometry_msgs::Twist body_speed;
  body_speed=wheel_invert(wheel_speed);

	float dt=1.0/20;
  body_position=update_pose(body_position, body_speed, dt);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp=ros::Time::now();
  odom_msg.header.frame_id=frame_id;
  odom_msg.twist.twist=body_speed;
  if(twist_covariance_.size()==6){
    odom_msg.twist.covariance[ 0] = twist_covariance_[0];
    odom_msg.twist.covariance[ 7] = twist_covariance_[1];
    odom_msg.twist.covariance[14] = twist_covariance_[2];
    odom_msg.twist.covariance[21] = twist_covariance_[3];
    odom_msg.twist.covariance[28] = twist_covariance_[4];
    odom_msg.twist.covariance[35] = twist_covariance_[5];
  }
  else ROS_ERROR_THROTTLE(5, "NOT IN %i", (int)twist_covariance_.size());

  if(pose_covariance_.size()==6){
    odom_msg.pose.covariance[ 0] = pose_covariance_[0];
    odom_msg.pose.covariance[ 7] = pose_covariance_[1];
    odom_msg.pose.covariance[14] = pose_covariance_[2];
    odom_msg.pose.covariance[21] = pose_covariance_[3];
    odom_msg.pose.covariance[28] = pose_covariance_[4];
    odom_msg.pose.covariance[35] = pose_covariance_[5];
  }
  else ROS_ERROR_THROTTLE(5, "NOT IN %i", (int)pose_covariance_.size());
  odom_msg.pose.pose=body_position;
  odom_pub.publish(odom_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_omni_odom");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	//publish
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	//Subscribe
	//ros::Subscriber odometry0 = nh.subscribe("wheel0/state", 10, state0_callback); 
	//ros::Subscriber odometry1 = nh.subscribe("wheel1/state", 10, state1_callback); 
	//ros::Subscriber odometry2 = nh.subscribe("wheel2/state", 10, state2_callback); 

	pnh.getParam("wheel_base",    wheel_base);
	pnh.getParam("wheel_radius",  wheel_radius);
	pnh.getParam("publish_rate", publish_rate);
	pnh.getParam("frame_id", frame_id);
	pnh.getParam("pose_covariance", pose_covariance_);
	pnh.getParam("twist_covariance", twist_covariance_);

	body_position.orientation.w=1.0;

  message_filters::Subscriber<control_msgs::JointControllerState> state_sub0(nh, "wheel0/state", 10);
  message_filters::Subscriber<control_msgs::JointControllerState> state_sub1(nh, "wheel1/state", 10);
  message_filters::Subscriber<control_msgs::JointControllerState> state_sub2(nh, "wheel2/state", 10);
  message_filters::Synchronizer<StateSyncPolicy> sync(StateSyncPolicy(20), state_sub0, state_sub1, state_sub2);
  sync.registerCallback(&sync_callback);

	ros::Rate loop_rate(publish_rate);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp=ros::Time::now();
  odom_msg.header.frame_id=frame_id;
  //odom_msg.twist.twist=body_speed;
  if(twist_covariance_.size()==6){
    odom_msg.twist.covariance[ 0] = twist_covariance_[0];
    odom_msg.twist.covariance[ 7] = twist_covariance_[1];
    odom_msg.twist.covariance[14] = twist_covariance_[2];
    odom_msg.twist.covariance[21] = twist_covariance_[3];
    odom_msg.twist.covariance[28] = twist_covariance_[4];
    odom_msg.twist.covariance[35] = twist_covariance_[5];
  }
  else ROS_ERROR_THROTTLE(5, "NOT IN %i", (int)twist_covariance_.size());

  if(pose_covariance_.size()==6){
    odom_msg.pose.covariance[ 0] = pose_covariance_[0];
    odom_msg.pose.covariance[ 7] = pose_covariance_[1];
    odom_msg.pose.covariance[14] = pose_covariance_[2];
    odom_msg.pose.covariance[21] = pose_covariance_[3];
    odom_msg.pose.covariance[28] = pose_covariance_[4];
    odom_msg.pose.covariance[35] = pose_covariance_[5];
  }
  else ROS_ERROR_THROTTLE(5, "NOT IN %i", (int)pose_covariance_.size());
  odom_msg.pose.pose=body_position;
  odom_pub.publish(odom_msg);

	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
