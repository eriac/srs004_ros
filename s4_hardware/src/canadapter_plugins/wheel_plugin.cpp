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
    state_pub_ = nh_.advertise<control_msgs::JointControllerState>("state", 10);
    command_sub_ = nh_.subscribe("command", 10, canin_callback);

  }
  void sync(void){
    // request speed
    s4_msgs::CANCode c_code;
    c_code.com = 1;
    c_code.remote = true;    
    output(c_code);
  }
  void input(s4_msgs::CANCode code){
    if(code.com == 1){// voltage
      float data0 = code.data[0] + code.data[1] / 256.0;
      float data1 = code.data[2] + code.data[3] / 256.0;
      
      std_msgs::Float32 float_msg;
      float_msg.data = data0;
      voltage_pub_.publish(float_msg);
    }
  }
  void inputCallback(const std_msgs::Float64& float_msg){
		if(can_msg.com==1){
			int temp1=can_msg.data[0]<<24|can_msg.data[1]<<16|can_msg.data[2]<<8|can_msg.data[3]<<0;
			int temp2=can_msg.data[4]<<8|can_msg.data[5]<<0;
			float current_position=(temp1-0x10000000)/PPR*2*3.1415;
			float c_data=(temp2-0x1000)/256.0;

			static ros::Time last_time = ros::Time(0);
			ros::Time current_time = ros::Time::now();
			static float last_position = 0;

			control_msgs::JointControllerState state_msg;
			state_msg.header.stamp=current_time;
			if(last_time != ros::Time(0)){
				float position_diff = current_position - last_position;
				ros::Duration time_diff= current_time - last_time;
				if(time_diff < ros::Duration(0.03)){
					ROS_WARN("input come in %f[s] < 0.03", time_diff.toSec());
					return;
				}
				state_msg.process_value = position_diff / time_diff.toSec();
				//printf("cp:%f, lp:%f, ct:%f, lt:%f\n", current_position, last_position, current_time.toSec(), last_time.toSec());
				//printf("dp:%f, dy:%f\n", current_position-last_position, (current_time-last_time).toSec());
			}

			last_time = current_time;
			last_position = current_position;
			state_pub_.publish(state_msg);

			std_msgs::Float64 c_msg;
			c_msg.data=c_data;
			//current_pub_.publish(c_msg);
		}
	}
 
  }


  ros::Publisher state_pub_;
  ros::Subscriber command_sub_;

};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::WheelPlugin, s4_hardware::CANLinkPluginsBase)