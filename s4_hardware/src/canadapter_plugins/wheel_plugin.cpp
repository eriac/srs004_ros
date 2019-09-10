#include <pluginlib/class_list_macros.h>
#include <s4_hardware/canlink_plugins_base.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
// SRS004
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class WheelPlugin : public CANLinkPluginsBase{
public:
  void init(void){
    ppr_ = 0.0;
    pnh_.getParam("ppr", ppr_);
    state_pub_ = nh_.advertise<control_msgs::JointControllerState>(name_ + "/state", 10);
    current_pub_  = nh_.advertise<std_msgs::Float64>(name_ + "/current", 10);
    command_sub_ = nh_.subscribe(name_ + "/command", 10, &WheelPlugin::commandCallback, this);

    last_time_ = ros::Time(0);
		last_position_ = 0;
  }
  void sync(void){
    // request speed
    s4_msgs::CANCode c_code;
    c_code.com = 1;
    c_code.remote = true;
    output(c_code);
  }
  void input(s4_msgs::CANCode code){
		if(code.com==1){
			int temp0 = !(code.data[0] & 0x80) ? (unsigned int)(code.data[0]<<8|code.data[1]<<0) : (unsigned int)(0xffffffff<<16|code.data[0]<<8|code.data[1]<<0);
			int temp1 = !(code.data[2] & 0x80) ? (unsigned int)(code.data[2]<<8|code.data[3]<<0) : (unsigned int)(0xffffffff<<16|code.data[2]<<8|code.data[3]<<0);
			int temp2 = !(code.data[4] & 0x80) ? (unsigned int)(code.data[4]<<8|code.data[5]<<0) : (unsigned int)(0xffffffff<<16|code.data[4]<<8|code.data[5]<<0);
			int temp3 = !(code.data[6] & 0x80) ? (unsigned int)(code.data[6]<<8|code.data[7]<<0) : (unsigned int)(0xffffffff<<16|code.data[6]<<8|code.data[7]<<0);
			
      float target = (float)temp0*(2*3.1415/ppr_);
      float speed = (float)temp1*(2*3.1415/ppr_);
      float drive = (float)temp2/1024;
      float current = (float)temp3/1024;

			control_msgs::JointControllerState state_msg;
			state_msg.header.stamp = ros::Time::now();

			state_msg.process_value = speed;
      state_msg.set_point = target;
      state_msg.command = drive;

			state_pub_.publish(state_msg);

			std_msgs::Float64 c_msg;
			c_msg.data=current;
      current_pub_.publish(c_msg);
		}
  }
  void commandCallback(const std_msgs::Float64& float_msg){
    unsigned int target= (int)float_msg.data*(ppr_/(2*3.1415));
    s4_msgs::CANCode cancode;
    cancode.com=1;
    cancode.length=2;
    cancode.data[0]=(target>>8)&0xFF;
    cancode.data[1]=(target>>0)&0xFF;
    output(cancode);
	}
  ros::Publisher state_pub_;
  ros::Publisher current_pub_;
  ros::Subscriber command_sub_;
  double ppr_;

  ros::Time last_time_;
  double last_position_;
};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::WheelPlugin, s4_hardware::CANLinkPluginsBase)