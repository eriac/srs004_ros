#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <s4_hardware/CANCode.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <math.h>
#include <sstream>
#include <string>

ros::Publisher hit_pub;
ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;

void canin_callback(const s4_hardware::CANCode& can_msg){
  if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
    if(can_msg.com==1 /*&&can_msg.length==4*/){
      std_msgs::Int32 count;
      count.data = can_msg.data[0];
      if(count.data > 0){
        hit_pub.publish(count);
      }
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "phycon_master_actual");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.getParam("CAN_CH", CAN_CH);
  pn.getParam("CAN_ID", CAN_ID);

  //publish
  hit_pub = n.advertise<std_msgs::Int32>("hit", 10);
  canlink_pub = n.advertise<s4_hardware::CANCode>("CANLink_out", 10);

  //Subscribe
  ros::Subscriber canin_sub = n.subscribe("CANLink_in", 10, canin_callback); 

  float dt=1.0/20;
  ros::Rate loop_rate(10); 
  while (ros::ok()){
    s4_hardware::CANCode can_msg;
    can_msg.channel=CAN_CH;
    can_msg.id=CAN_ID;
    can_msg.com=1;
    can_msg.remote=true;
    canlink_pub.publish(can_msg);

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}