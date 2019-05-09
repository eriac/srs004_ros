#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <s4_msgs/Joy.h>
#include <string>

ros::Publisher joy_pub;
std::string joy_type = "none";
bool restamp =false;

#define PS3_Button_Max 17
#define PS3_Select 0
#define PS3_L3     1
#define PS3_R3     2
#define PS3_Start  3
#define PS3_Up     4
#define PS3_Right  5
#define PS3_Down   6
#define PS3_Left   7
#define PS3_L2     8
#define PS3_R2     9
#define PS3_L1    10
#define PS3_R1    11
#define PS3_triangle 12
#define PS3_circle   13
#define PS3_cross    14
#define PS3_square   15
#define PS3_PS 16

#define PS3_LH 0
#define PS3_LV 1
#define PS3_RH 2
#define PS3_RV 3
#define PS3_AL2 12
#define PS3_AR2 13
s4_msgs::Joy convert_ps3(const sensor_msgs::Joy& joy_msg){
  s4_msgs::Joy joy_out;
  joy_out.header = joy_msg.header;
  joy_out.left.AV = joy_msg.axes[1];
  joy_out.left.AH = joy_msg.axes[0];
  joy_out.left.B1 = (1 - joy_msg.axes[14]) / 2 * joy_msg.buttons[10];
  joy_out.left.B2 = (1 - joy_msg.axes[12]) / 2 * joy_msg.buttons[8];
  joy_out.left.B3 = joy_msg.buttons[1];
  joy_out.left.CU = joy_msg.buttons[4];//8
  joy_out.left.CD = joy_msg.buttons[6];//10
  joy_out.left.CL = joy_msg.buttons[7];//11? nw
  joy_out.left.CR = joy_msg.buttons[5];//9
  joy_out.right.AV = joy_msg.axes[3];
  joy_out.right.AH = joy_msg.axes[2];
  joy_out.right.B1 = (1 - joy_msg.axes[15]) / 2 * joy_msg.buttons[11];
  joy_out.right.B2 = (1 - joy_msg.axes[13]) / 2 * joy_msg.buttons[9];
  joy_out.right.B3 = joy_msg.buttons[2];
  joy_out.right.CU = (1 - joy_msg.axes[16]) / 2 * joy_msg.buttons[12];
  joy_out.right.CD = (1 - joy_msg.axes[18]) / 2 * joy_msg.buttons[14];
  joy_out.right.CL = (1 - joy_msg.axes[19]) / 2 * joy_msg.buttons[15];
  joy_out.right.CR = (1 - joy_msg.axes[17]) / 2 * joy_msg.buttons[13];
  joy_out.other.resize(2);
  joy_out.other[0] = joy_msg.buttons[0];
  joy_out.other[1] = joy_msg.buttons[3];
  return joy_out;
}

s4_msgs::Joy convert_iphone(const sensor_msgs::Joy& joy_msg){
  s4_msgs::Joy joy_out;
  joy_out.header = joy_msg.header;
  joy_out.left.AV = joy_msg.axes[1];
  joy_out.left.AH = -joy_msg.axes[0];
  joy_out.left.B1 = joy_msg.axes[10];
  joy_out.left.B2 = joy_msg.axes[12];
  joy_out.left.B3 = 0;
  joy_out.left.CU = joy_msg.axes[5] > 0 ? 1 : 0;
  joy_out.left.CD = joy_msg.axes[5] < 0 ? 1 : 0;
  joy_out.left.CL = joy_msg.axes[4] < 0 ? 1 : 0;
  joy_out.left.CR = joy_msg.axes[4] > 0 ? 1 : 0;
  joy_out.right.AV = joy_msg.axes[3];
  joy_out.right.AH = -joy_msg.axes[2];
  joy_out.right.B1 = joy_msg.axes[11];
  joy_out.right.B2 = joy_msg.axes[13];
  joy_out.right.B3 = 0;
  joy_out.right.CU = joy_msg.axes[9];
  joy_out.right.CD = joy_msg.axes[6];
  joy_out.right.CL = joy_msg.axes[8];
  joy_out.right.CR = joy_msg.axes[7];
  return joy_out;
}

s4_msgs::Joy convert_elecom(const sensor_msgs::Joy& joy_msg){
  s4_msgs::Joy joy_out;
  joy_out.header = joy_msg.header;
  joy_out.left.AV = joy_msg.axes[1];
  joy_out.left.AH = joy_msg.axes[0];
  joy_out.left.B1 = joy_msg.buttons[4];
  joy_out.left.B2 = joy_msg.buttons[6];
  joy_out.left.B3 = joy_msg.buttons[8];
  joy_out.left.CU = joy_msg.axes[5] == 1 ? 1 : 0;
  joy_out.left.CD = joy_msg.axes[5] == -1 ? 1 : 0;
  joy_out.left.CL = joy_msg.axes[4] == 1 ? 1 : 0;
  joy_out.left.CR = joy_msg.axes[4] == -1 ? 1 : 0;
  joy_out.right.AV = joy_msg.axes[3];
  joy_out.right.AH = joy_msg.axes[2];
  joy_out.right.B1 = joy_msg.buttons[5];
  joy_out.right.B2 = joy_msg.buttons[7];
  joy_out.right.B3 = joy_msg.buttons[9];
  joy_out.right.CU = joy_msg.buttons[1];
  joy_out.right.CD = joy_msg.buttons[2];
  joy_out.right.CL = joy_msg.buttons[0];
  joy_out.right.CR = joy_msg.buttons[3];
  joy_out.other.resize(2);
  joy_out.other[0] = joy_msg.buttons[10];
  joy_out.other[1] = joy_msg.buttons[11];
  return joy_out;
}

void joy_callback(const sensor_msgs::Joy& joy_msg){
  s4_msgs::Joy joy_out;
  if(joy_type == "elecom"){
    joy_out = convert_elecom(joy_msg);
  }
  else if(joy_type == "ps3"){
    joy_out = convert_ps3(joy_msg);
  }
  else if(joy_type == "iphone"){
    joy_out = convert_iphone(joy_msg);
  }
  else{
    ROS_WARN_THROTTLE(5.0, "joy_type is illegal");
    return;
  }

  if(restamp){
    joy_out.header.stamp = ros::Time::now();
  }
  joy_pub.publish(joy_out);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_operation_test_app");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("joy_type", joy_type);
  pnh.getParam("restamp", restamp);

  joy_pub = nh.advertise<s4_msgs::Joy>("standard_joy", 10);
  ros::Subscriber joy_sub = nh.subscribe("joy", 1, joy_callback);
  
  ros::spin();
  return 0;
}