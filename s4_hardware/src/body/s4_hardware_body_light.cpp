#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <s4_hardware/CANCode.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <math.h>
#include <sstream>
#include <string>
#include <algorithm>

ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;

void light_callback(const std_msgs::Float64& float_msg){
  int value = std::min(std::max((int)(float_msg.data * 255), 0), 255);

  s4_hardware::CANCode can_msg;
  can_msg.channel=CAN_CH;
  can_msg.id=CAN_ID;
  can_msg.data[0] = value;
  can_msg.length = 1;
  can_msg.com=2;
  can_msg.remote=false;
  canlink_pub.publish(can_msg);  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "phycon_master_actual");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.getParam("CAN_CH", CAN_CH);
  pn.getParam("CAN_ID", CAN_ID);

  //publish
  canlink_pub  = n.advertise<s4_hardware::CANCode>("CANLink_out", 1000);

  //Subscribe
  ros::Subscriber light_sub = n.subscribe("intensity", 10, light_callback); 

  ros::spin();
  return 0;
}