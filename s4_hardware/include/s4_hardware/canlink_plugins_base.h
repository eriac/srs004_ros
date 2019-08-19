#ifndef _S4_HARDWARE_CANLINK_PLUGINS_BASE_
#define _S4_HARDWARE_CANLINK_PLUGINS_BASE_

// OSS
#include <string>
#include <boost/bind.hpp>
#include <boost/function.hpp>
// ROS
#include <ros/ros.h>
// SRS004
#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class CANLinkPluginsBase{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle pnh, std::string channel, int id, boost::function<bool(s4_msgs::CANCode)> func);
  virtual void init(void) = 0;
  virtual void sync(void) = 0;
  void inputCode(s4_msgs::CANCode code);
  virtual void input(s4_msgs::CANCode code) = 0;
  void output(s4_msgs::CANCode code);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string channel_;
  int id_;
  boost::function<bool(s4_msgs::CANCode)> outputCode_;
};

void CANLinkPluginsBase::initialize(ros::NodeHandle nh, ros::NodeHandle pnh, std::string channel, int id, boost::function<bool(s4_msgs::CANCode)> func){
  nh_ = nh;
  pnh_ = pnh;
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
#endif