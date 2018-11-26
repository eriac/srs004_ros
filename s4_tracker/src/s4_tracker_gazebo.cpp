#include "ros/ros.h"

#include "math.h"
#include "gazebo_msgs/ModelStates.h"
#include "nav_msgs/Odometry.h"

#include <string>
#include <iostream>
#include <sstream>

ros::Publisher odom_pub;
std::string model_name;
std::string frame_id;
void models_callback(const gazebo_msgs::ModelStates& model_msg){
int model_size=model_msg.name.size();
  for(int i=0;i<model_size;i++){
    if(model_msg.name[i]==model_name){
      nav_msgs::Odometry odom;
      odom.header.frame_id=frame_id;
      odom.header.stamp=ros::Time::now();
      odom.pose.pose=model_msg.pose[i];
      odom.twist.twist=model_msg.twist[i];
      odom_pub.publish(odom);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_model_base_publisher");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  //rosparam
  pn.getParam("model_name",  model_name);
  pn.getParam("frame_id", frame_id);
  //publisher
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  //subscriibe
  ros::Subscriber model_sub   = n.subscribe("/gazebo/model_states", 1, models_callback);

  ros::spin();
  return 0;
}
