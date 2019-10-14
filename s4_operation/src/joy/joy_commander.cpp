// OSS
#include <string>
// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
//SRS004
#include <s4_msgs/joy_converter.h>
#include <s4_msgs/joy_accessor.h>
#include <s4_msgs/Joy.h>

class JoyCommander{
public:
  JoyCommander(): nh_(), pnh_("~"){
    standard_joy_pub_ = nh_.advertise<s4_msgs::Joy>("standard_joy", 10);
    nav_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    gun_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("gun_cmd_vel", 10);

    gun_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("gun_cmd_vel", 10);
    gun_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("gun_cmd_vel", 10);
    gun_laser_pub_ = nh_.advertise<std_msgs::Bool>("gun_laser", 10);
    gun_shot_pub_ = nh_.advertise<std_msgs::Int32>("gun_shot", 10);

    ps3_joy_sub_ = nh_.subscribe("ps3_joy", 10, &JoyCommander::ps3Callback, this);
    joy_converter_ps3_.setDevice("ps3");
  }

  void ps3Callback(const sensor_msgs::Joy& joy_msg){
    s4_msgs::Joy std_joy = joy_converter_ps3_.convert(joy_msg);
    joy_accesor_.UpdateJoy(std_joy);

    geometry_msgs::Twist nav_twist;
    nav_twist.linear.x = 0.4 * std_joy.left.AV;
    // nav_twist.linear.y = 0.4 * std_joy.left.AH;
    // nav_twist.angular.z = 1.5 * std_joy.right.AH;
    nav_twist.angular.z = 2.5 * std_joy.left.AH;

    geometry_msgs::Twist gun_twist;
    gun_twist.linear.y = 0.2 * std_joy.right.AH;
    gun_twist.linear.z = 0.2 * std_joy.right.AV;

    if(joy_accesor_.GetPress().right.CD){
      ROS_INFO("press right CD");
      std_msgs::Int32 int_msg;
      int_msg.data=3;
      gun_shot_pub_.publish(int_msg);
    }

    if(joy_accesor_.GetPress().right.CL){
      ROS_INFO("press right CL");
      std_msgs::Bool bool_msg;
      bool_msg.data=true;
      gun_laser_pub_.publish(bool_msg);
    }
    else if(joy_accesor_.GetPress().right.CU){
      ROS_INFO("press right CU");
      std_msgs::Bool bool_msg;
      bool_msg.data=false;
      gun_laser_pub_.publish(bool_msg);
    }

    standard_joy_pub_.publish(std_joy);
    nav_twist_pub_.publish(nav_twist);
    gun_twist_pub_.publish(gun_twist);

    joy_accesor_.ResetChange();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher standard_joy_pub_;

  ros::Publisher nav_twist_pub_;
  ros::Publisher gun_twist_pub_;
  ros::Publisher gun_laser_pub_;
  ros::Publisher gun_shot_pub_;

  ros::Subscriber ps3_joy_sub_;

  s4_msgs::JoyConverter joy_converter_ps3_;
  s4_msgs::JoyAccesor joy_accesor_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "joy_commander");
  //pnh.getParam("restamp", restamp);
  JoyCommander joy_commander;
  ros::spin();
  return 0;
}