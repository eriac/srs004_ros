#include <pluginlib/class_list_macros.h>
#include <s4_hardware/canlink_plugins_base.h>

#include <ros/ros.h>
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class TestPlugin : public CANLinkPluginsBase{
public:
  void sync(void);
  void input(s4_msgs::CANCode code){
    ROS_INFO("input occur");
  }
};
}
PLUGINLIB_EXPORT_CLASS(s4_hardware::TestPlugin, s4_hardware::CANLinkPluginsBase)