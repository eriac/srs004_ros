#include <pluginlib/class_list_macros.h>
#include <s4_hardware/canlink_plugins_base.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class MasterPlugin : public CANLinkPluginsBase{
public:
  void init(void){
    voltage_pub_ = nh_.advertise<std_msgs::Float32>("voltage", 10);
  }
  void sync(void){
    // request voltage
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
  ros::Publisher voltage_pub_;
};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::MasterPlugin, s4_hardware::CANLinkPluginsBase)