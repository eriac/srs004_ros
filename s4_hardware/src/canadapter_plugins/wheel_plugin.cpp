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
    ppr_ = 1.0;
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
			int temp1 = code.data[0]<<24|code.data[1]<<16|code.data[2]<<8|code.data[3]<<0;
			int temp2 = code.data[4]<<8|code.data[5]<<0;
			int temp3 = code.data[6]<<8|code.data[7]<<0;
			float current_position = -(temp1-0x10000000)/ppr_*2*3.1415;
			float c_data = (temp2-0x1000)/256.0;
      float current_target = -(temp3-0x1000)*2*3.1415/102.1/0x1000*4000/ppr_*(2*3.14*20);

			ros::Time current_time = ros::Time::now();
			control_msgs::JointControllerState state_msg;
			state_msg.header.stamp=current_time;
			if(last_time_ != ros::Time(0)){
				float position_diff = current_position - last_position_;
				ros::Duration time_diff= current_time - last_time_;
				if(time_diff < ros::Duration(0.03)){
					ROS_WARN("input come in %f[s] < 0.03", time_diff.toSec());
					return;
				}
				state_msg.process_value = position_diff / time_diff.toSec();
        state_msg.set_point = current_target;
				//printf("cp:%f, lp:%f, ct:%f, lt:%f\n", current_position, last_position, current_time.toSec(), last_time.toSec());
				//printf("dp:%f, dy:%f\n", current_position-last_position, (current_time-last_time).toSec());
			}

			last_time_ = current_time;
			last_position_ = current_position;
			state_pub_.publish(state_msg);

			std_msgs::Float64 c_msg;
			c_msg.data=c_data;
      current_pub_.publish(c_msg);
		}
  }
  void commandCallback(const std_msgs::Float64& float_msg){
  	float target= -float_msg.data*ppr_/(2*3.14*20);
	  int data=0x1000+(target/2/3.1415*102.1)*0x1000/4000;
    s4_msgs::CANCode cancode;
    cancode.com=1;
    cancode.length=2;
    cancode.data[0]=(data>>8)&0xFF;
    cancode.data[1]=(data>>0)&0xFF;
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