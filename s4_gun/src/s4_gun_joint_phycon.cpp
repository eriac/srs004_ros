#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "s4_comport/CANCode.h"

ros::Publisher canlink_pub;
ros::Publisher joint_pub;
std::string CAN_CH="";
int CAN_ID=0;
float z_ajust=0;
float y_ajust=0;

float last_yaw=0;
float last_pitch=0;
void publish_joint(void){
  int servo1=1520+(last_yaw  +z_ajust)/(2*3.14)*360*10;
  int servo2=1520-(last_pitch+y_ajust)/(2*3.14)*360*10;

  s4_comport::CANCode cancode;
  cancode.channel=CAN_CH;
  cancode.id=CAN_ID;
  cancode.com=1;
  cancode.length=4;
  cancode.data[0]=(servo1>>8)&0xFF;
  cancode.data[1]=(servo1>>0)&0xFF;
  cancode.data[2]=(servo2>>8)&0xFF;
  cancode.data[3]=(servo2>>0)&0xFF;
  canlink_pub.publish(cancode);
}

void yaw_callback(const std_msgs::Float64& float_msg){
  last_yaw=float_msg.data;
  publish_joint();
}
void pitch_callback(const std_msgs::Float64& float_msg){
  last_pitch=float_msg.data;
  publish_joint();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pycon_gun_joint_phycon");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.getParam("CAN_CH", CAN_CH);
  pn.getParam("CAN_ID", CAN_ID);
  pn.getParam("y_ajust", y_ajust);
  pn.getParam("z_ajust", z_ajust);

  //publish
  canlink_pub = n.advertise<s4_comport::CANCode>("CANLink_out", 10);

  //subscriibe
  ros::Subscriber yaw_sub    = n.subscribe("gun_yaw/command", 10, yaw_callback);
  ros::Subscriber pitch_sub  = n.subscribe("gun_pitch/command", 10, pitch_callback);

  ros::Duration(1.0).sleep();
  ros::Rate loop_rate(20); 
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
