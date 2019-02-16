#include <ros/ros.h>
#include <s4_msgs/gameAppAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/Joy.h"

#define PS3_Button_Max 17
#define PS3_Select 0
#define PS3_L3     1
#define PS3_R3     2
#define PS3_Start  3
#define PS3_Up     4
#define PS3_Right  5
#define PS3_Down   6
#define PS3_Left   7
#define PS3_L2     8
#define PS3_R2     9
#define PS3_L1    10
#define PS3_R1    11
#define PS3_triangle 12
#define PS3_circle   13
#define PS3_cross    14
#define PS3_square   15
#define PS3_PS 16
#define PS3_LH 0
#define PS3_LV 1
#define PS3_RH 2
#define PS3_RV 3

typedef actionlib::SimpleActionClient<s4_msgs::gameAppAction> Client;

float scale_linear_x=0.3;
float scale_linear_y=0.3;
float scale_angular_z=2.5;

geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist aim_vel;
void joy_callback(const sensor_msgs::Joy& joy_msg){
  cmd_vel.linear.x =joy_msg.axes[PS3_LV]*scale_linear_x;
  cmd_vel.linear.y =joy_msg.axes[PS3_LH]*scale_linear_y;
  cmd_vel.angular.z=joy_msg.axes[PS3_RH]*scale_angular_z;

  if     (joy_msg.buttons[PS3_Right])aim_vel.angular.z=1;
  else if(joy_msg.buttons[PS3_Left]) aim_vel.angular.z=-1;
  else aim_vel.angular.z=0;

  if     (joy_msg.buttons[PS3_Up])  aim_vel.angular.y=1;
  else if(joy_msg.buttons[PS3_Down])aim_vel.angular.y=-1;
  else aim_vel.angular.y=0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_operation_test_app");
  ros::NodeHandle nh;
  ROS_INFO("START");
  Client move_client("move_action", false); // true -> don't need ros::spin()
  Client turret_client("turret_action", false); // true -> don't need ros::spin()

  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  ros::Rate loop_rate(10);
  
  while (ros::ok()){
    //move
    if(move_client.isServerConnected()){
      ROS_INFO_ONCE("move server connected");
      s4_msgs::gameAppGoal goal1;
      goal1.mode=3;
      goal1.header.stamp=ros::Time::now();
      goal1.twist=cmd_vel;
      goal1.duration=3.0;
      move_client.sendGoal(goal1);
    }
    //turret
    if(turret_client.isServerConnected()){
      ROS_INFO_ONCE("turret server connected");
      s4_msgs::gameAppGoal goal2;
      goal2.mode=3;
      goal2.header.stamp=ros::Time::now();
      goal2.twist=aim_vel;
      goal2.duration=3.0;
      turret_client.sendGoal(goal2);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}