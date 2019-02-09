#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>

#include "math.h"
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "s4_description_joint_publisher");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  //publish
  ros::Publisher joint_pub   = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Rate loop_rate(30);
  while (ros::ok()){
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(32);
    js0.position.resize(32);
    for(int i=0;i<3;i++){  
      js0.name[10*i+0]="omni0/wheel" + std::to_string(i) + "/sus_joint";
      js0.name[10*i+1]="omni0/wheel" + std::to_string(i)+  "/wheel_joint";
      js0.position[10*i+0]=0;
      js0.position[10*i+1]=0;
      for(int j=0;j<8;j++){
        js0.name[10*i+2+j]="omni0/wheel" + std::to_string(i) + "/barrel_" + std::to_string(j) + "_joint";
        js0.position[10*i+2+j]=0;
      }
    }
    js0.name[30]="gun0/base2_joint";
    js0.name[31]="gun0/gun_joint";
    js0.position[30]=0;
    js0.position[31]=0;
    joint_pub.publish(js0);

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
