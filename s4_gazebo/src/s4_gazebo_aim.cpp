#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>

#include "math.h"
#include <sstream>
#include <string>

float train_target=0.0;
float elevation_target=0.0;
void joint_callback(const sensor_msgs::JointState& joint_msg){
  if(joint_msg.name.size()==joint_msg.position.size()){
    for(int i=0;i<(int)joint_msg.name.size();i++){
      if(joint_msg.name[i]=="train")train_target=joint_msg.position[i];
      else if(joint_msg.name[i]=="elevation")elevation_target=joint_msg.position[i];
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "s4_gazebo_aim");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //publish
	ros::Publisher train_pub     = nh.advertise<std_msgs::Float64>("train", 10);
	ros::Publisher elevation_pub = nh.advertise<std_msgs::Float64>("elevation", 10);

  //subscribe
  ros::Subscriber joint_sub = nh.subscribe("joints", 10, joint_callback); 
	
  ros::Rate loop_rate(10);
  while (ros::ok()){
    std_msgs::Float64 train_msg;
    std_msgs::Float64 elevation_msg;
    train_msg.data=train_target;
    elevation_msg.data=elevation_target;
    train_pub.publish(train_msg);
    elevation_pub.publish(elevation_msg);
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
