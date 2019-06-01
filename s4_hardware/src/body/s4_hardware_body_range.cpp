#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <s4_hardware/CANCode.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/Range.h>

#include <math.h>
#include <sstream>
#include <string>

ros::Publisher range0_pub;
ros::Publisher range1_pub;
ros::Publisher range2_pub;
ros::Publisher canlink_pub;
std::string CAN_CH="";
int CAN_ID=0;
std::string tf_prefix="";


void canin_callback(const s4_hardware::CANCode& can_msg){
  if(can_msg.channel==CAN_CH && can_msg.id==CAN_ID){
    if(can_msg.com==3 /*&&can_msg.length==4*/){
      int data0=(can_msg.data[0] << 8) | can_msg.data[1];
      int data1=(can_msg.data[2] << 8) | can_msg.data[3];
      int data2=(can_msg.data[4] << 8) | can_msg.data[5];

      ros::Time ros_time_now = ros::Time::now();

      sensor_msgs::Range range0;
      range0.header.stamp = ros_time_now;
      range0.header.frame_id = tf_prefix + "/sensor0/range0_link";
      range0.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range0.field_of_view = 0.5;
      range0.min_range = 0.05;
      range0.max_range = 4.0;
      range0.range = (float)data0 / 1000.0;
      range0_pub.publish(range0);

      sensor_msgs::Range range1;
      range1.header.stamp = ros_time_now;
      range1.header.frame_id = tf_prefix + "/sensor0/range1_link";
      range1.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range1.field_of_view = 0.5;
      range1.min_range = 0.05;
      range1.max_range = 4.0;
      range1.range = (float)data1 / 1000.0;
      range1_pub.publish(range1);

      sensor_msgs::Range range2;
      range2.header.stamp = ros_time_now;
      range2.header.frame_id = tf_prefix + "/sensor0/range2_link";
      range2.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range2.field_of_view = 0.5;
      range2.min_range = 0.05;
      range2.max_range = 4.0;
      range2.range = (float)data2 / 1000.0;
      range2_pub.publish(range2);
    }
  }
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "phycon_master_actual");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.getParam("CAN_CH", CAN_CH);
  pn.getParam("CAN_ID", CAN_ID);
  pn.getParam("tf_prefix", tf_prefix);

  //publish
  range0_pub = n.advertise<sensor_msgs::Range>("range0", 10);
  range1_pub = n.advertise<sensor_msgs::Range>("range1", 10);
  range2_pub = n.advertise<sensor_msgs::Range>("range2", 10);
  canlink_pub  = n.advertise<s4_hardware::CANCode>("CANLink_out", 10);

  //Subscribe
  ros::Subscriber canin_sub     = n.subscribe("CANLink_in", 10, canin_callback); 

  //Diagnostic
  diagnostic_updater::Updater updater;
  updater.setHardwareID("MainModule");
  updater.add("Active", diagnostic0);

  float dt=1.0/20;
  ros::Rate loop_rate(10); 
  while (ros::ok()){
    s4_hardware::CANCode can_msg;
    can_msg.channel=CAN_CH;
    can_msg.id=CAN_ID;
    can_msg.com=3;
    can_msg.remote=true;
    canlink_pub.publish(can_msg);

    updater.update();
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}