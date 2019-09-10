// OSS
#include <string>
// ROS
#include <ros/ros.h>
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

    ps3_joy_sub_ = nh_.subscribe("ps3_joy", 10, &JoyCommander::ps3Callback, this);

    joy_converter_ps3.setDevice("ps3");
  }

  void ps3Callback(const sensor_msgs::Joy& joy_msg){
    s4_msgs::Joy std_joy = joy_converter_ps3.convert(joy_msg);

    geometry_msgs::Twist nav_twist;
    nav_twist.linear.x = 0.4 * std_joy.left.AV;
    // nav_twist.linear.y = 0.4 * std_joy.left.AH;
    // nav_twist.angular.z = 1.5 * std_joy.right.AH;
    nav_twist.angular.z = 2.5 * std_joy.left.AH;

    geometry_msgs::Twist gun_twist;
    gun_twist.linear.y = 0.2 * std_joy.right.AH;
    gun_twist.linear.z = 0.2 * std_joy.right.AV;

    standard_joy_pub_.publish(std_joy);
    nav_twist_pub_.publish(nav_twist);
    gun_twist_pub_.publish(gun_twist);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher standard_joy_pub_;

  ros::Publisher nav_twist_pub_;
  ros::Publisher gun_twist_pub_;

  ros::Subscriber ps3_joy_sub_;

  s4_msgs::JoyConverter joy_converter_ps3;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "joy_commander");
  //pnh.getParam("restamp", restamp);
  JoyCommander joy_commander;
  ros::spin();
  return 0;
}