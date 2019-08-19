#include <pluginlib/class_list_macros.h>
#include <s4_hardware/canlink_plugins_base.h>

#include <ros/ros.h>
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class TestPlugin : public CANLinkPluginsBase{
public:
  void init(void){
  }

  void sync(void){
    ROS_INFO("sync occur");
    
    s4_msgs::CANCode c_code;
    c_code.channel = "S";
    c_code.id = 1;
    c_code.com = 1;
    c_code.remote = true;

    output(c_code);
    return;
  }
  void input(s4_msgs::CANCode code){
    ROS_INFO("input occur");
  }
};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::TestPlugin, s4_hardware::CANLinkPluginsBase)