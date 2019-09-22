#include <pluginlib/class_list_macros.h>
#include <s4_hardware/canlink_plugins_base.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/JointControllerState.h>
// SRS004
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class GunPlugin : public CANLinkPluginsBase{
public:
  void init(void){
    state_pub_ = nh_.advertise<geometry_msgs::PointStamped>(name_ + "/carriage/state", 10);
    set_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(name_ + "/carriage/set_point", 10);
    //current_pub_  = nh_.advertise<std_msgs::Float64>(name_ + "/current", 10);
    command_sub_ = nh_.subscribe(name_ + "/carriage/command", 10, &GunPlugin::commandCallback, this);

    last_time_ = ros::Time(0);
		last_position_ = 0;
    train_trim_ = 0;
    elevation_trim_ = 0;
  }
  void sync(void){
    // request speed
    s4_msgs::CANCode c_code;
    c_code.com = 1;
    c_code.remote = true;
    output(c_code);
  }
  void input(s4_msgs::CANCode code){
    if(code.com==1 /*&&code.length==6*/){
      int temp1=code.data[0]<<8|code.data[1]<<0;
      int temp2=code.data[2]<<8|code.data[3]<<0;
      int temp3=code.data[4]<<8|code.data[5]<<0;
      int temp4=code.data[6]<<8|code.data[7]<<0;
      float set_point1 = -(temp1-7500)*3.1415/5333;
      float set_point2 = (temp2-7500)*3.1415/5333;

      geometry_msgs::PointStamped set_point_msg;
      set_point_msg.point.x = 1.0;
      set_point_msg.point.y = set_point_msg.point.x * tan(set_point1);
      float set_holizon = sqrt(set_point_msg.point.x * set_point_msg.point.x + set_point_msg.point.y * set_point_msg.point.y);
      set_point_msg.point.z = set_holizon * tan(set_point2);
      //ROS_INFO("t: %f(%i), e:%f(%i)", joint1, temp1, joint2, temp2);
      set_point_pub_.publish(set_point_msg);

      float joint1 = -(temp3-7500)*3.1415/5333;
      float joint2 = (temp4-7500)*3.1415/5333;

      geometry_msgs::PointStamped point_msg;
      point_msg.point.x = 1.0;
      point_msg.point.y = point_msg.point.x * tan(joint1);
      float holizon = sqrt(point_msg.point.x * point_msg.point.x + point_msg.point.y * point_msg.point.y);
      point_msg.point.z = holizon * tan(joint2);
      //ROS_INFO("t: %f(%i), e:%f(%i)", joint1, temp1, joint2, temp2);
      state_pub_.publish(point_msg);
    }
  }
  void commandCallback(const geometry_msgs::PointStamped& point_msg){
    float holizon = sqrt(point_msg.point.x * point_msg.point.x + point_msg.point.y * point_msg.point.y);
    if(holizon > 0.1){
      float train_target = atan2(point_msg.point.y, point_msg.point.x);
      float elevation_target = atan2(point_msg.point.z, holizon);
      // if(train_min < train_target && train_target < train_max && elevation_min < elevation_target && elevation_target < elevation_max){
      // }
      int train_command = -train_target*5333/3.1415+7500+train_trim_;
      int elevation_command = elevation_target*5333/3.1415+7500+elevation_trim_;
      //ROS_WARN("t: %f(%i), e:%f(%i)", train_target, train_command, elevation_target, elevation_command);

      s4_msgs::CANCode cancode;
      cancode.com=1;
      cancode.length=4;
      cancode.data[0]=(train_command>>8)&0xFF;
      cancode.data[1]=(train_command>>0)&0xFF;
      cancode.data[2]=(elevation_command>>8)&0xFF;
      cancode.data[3]=(elevation_command>>0)&0xFF;
      output(cancode);
    }
  }

  ros::Publisher state_pub_;
  ros::Publisher set_point_pub_;
  ros::Publisher current_pub_;
  ros::Subscriber command_sub_;
  double ppr_;

  ros::Time last_time_;
  double last_position_;
  int train_trim_;
  int elevation_trim_;
};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::GunPlugin, s4_hardware::CANLinkPluginsBase)