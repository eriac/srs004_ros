#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <s4_hardware/CANCode.h>
#include <control_msgs/JointControllerState.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <cmath>

ros::Publisher  joint_pub;
ros::Publisher  point_pub;
ros::Publisher  canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;
float PPR=400;
int diagnostic_counter=0;

float train_min = -0.6;
float train_max = 0.6;
float elevation_min = -0.18;
float elevation_max = 0.17;//0.12
int elevation_trim = 0;
int train_trim = 0;

float train_target=0.0;
float elevation_target=0.0;
void point_callback(const geometry_msgs::Point& point_msg){
  float holizon = sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
  if(holizon > 0.1){
    train_target = atan2(point_msg.y, point_msg.x);
    elevation_target = atan2(point_msg.z, holizon);
    if(train_min < train_target && train_target < train_max && elevation_min < elevation_target && elevation_target < elevation_max){
      int train_command = -train_target*5333/3.1415+7500+train_trim;
      int elevation_command = elevation_target*8000/3.1415+7500+elevation_trim;
      //ROS_INFO("t: %f(%i), e:%f(%i)", train_target, train_command, elevation_target, elevation_command);

      s4_hardware::CANCode cancode;
      cancode.channel=CAN_CH;
      cancode.id=CAN_ID;
      cancode.com=1;
      cancode.length=4;
      cancode.data[0]=(train_command>>8)&0xFF;
      cancode.data[1]=(train_command>>0)&0xFF;
      cancode.data[2]=(elevation_command>>8)&0xFF;
      cancode.data[3]=(elevation_command>>0)&0xFF;
      canlink_pub.publish(cancode);
    }
  }
}

void canlink_callback(const s4_hardware::CANCode& can_msg){
  if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
    if(can_msg.com==1 /*&&can_msg.length==6*/){
      int temp1=can_msg.data[0]<<8|can_msg.data[1]<<0;
      int temp2=can_msg.data[2]<<8|can_msg.data[3]<<0;
      float joint1 = -(temp1-7500)*3.1415/5333;
      float joint2 = (temp2-7500)*3.1415/5333;

      sensor_msgs::JointState joint_msg;
      joint_msg.header.stamp = ros::Time::now();
      joint_msg.name.resize(2);
      joint_msg.name[0]="gun0/base2_joint";
      joint_msg.name[1]="gun0/gun_joint";
      joint_msg.position.resize(2);
      joint_msg.position[0] = joint1;
      joint_msg.position[1] = -joint2;
      joint_pub.publish(joint_msg);
      //diagnostic_counter=0;
    }
  }
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if(diagnostic_counter<10){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
  }
  else{
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Response.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pycon_wheel_actual");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("CAN_CH", CAN_CH);
  pnh.getParam("CAN_ID", CAN_ID);

  pnh.getParam("elevation_trim", elevation_trim);
  pnh.getParam("train_trim", train_trim);

  //publish
  point_pub  = nh.advertise<geometry_msgs::Point>("state", 10);
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  canlink_pub  = nh.advertise<s4_hardware::CANCode>("CANLink_out", 10);
  
  //subscribe
  ros::Subscriber point_sub = nh.subscribe("command", 10, point_callback); 
  ros::Subscriber canlink_sub = nh.subscribe("CANLink_in", 10, canlink_callback);
  
  //Diagnostic
  //diagnostic_updater::Updater updater;
  //updater.setHardwareID("WheelModule");
  //updater.add("Active", diagnostic0);


  ros::Duration(1.0).sleep();

  ros::Rate loop_rate(10); 
  while (ros::ok()){

    s4_hardware::CANCode cancode;
    cancode.channel=CAN_CH;
    cancode.id=CAN_ID;
    cancode.com=1;
    cancode.remote=true;
    canlink_pub.publish(cancode);

    //updater.update();
    //diagnostic_counter++;

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
