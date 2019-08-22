#include <s4_hardware/canlink_plugins_base.h>
// OSS
#include <string>
#include <boost/bind.hpp>
#include <boost/function.hpp>
// ROS
#include <ros/ros.h>
// SRS004
#include <s4_msgs/CANCode.h>

namespace s4_hardware{

void CANLinkPluginsBase::initialize(ros::NodeHandle nh, ros::NodeHandle pnh, std::string name, std::string channel, int id, boost::function<bool(s4_msgs::CANCode)> func){
  nh_ = nh;
  pnh_ = pnh;
  name_ = name;
  channel_ = channel;
  id_ = id;
  outputCode_ = func;

  // user initialize
  init();
}

void CANLinkPluginsBase::inputCode(s4_msgs::CANCode code){
  if(code.channel == channel_ && code.id == id_){
    input(code);
  }
}

void CANLinkPluginsBase::output(s4_msgs::CANCode code){
  code.channel = channel_;
  code.id = id_;
  outputCode_(code);
}

}
