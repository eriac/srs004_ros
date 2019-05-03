#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <algorithm>

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

typedef actionlib::SimpleActionClient<s4_msgs::GameAppAction> Client;

float scale_linear_x=0.3;
float scale_linear_y=0.3;
float scale_angular_z=2.5;

std::vector<int> buttons_press;
std::vector<int> buttons_release;
sensor_msgs::Joy last_joy_msg;
void joy_callback(const sensor_msgs::Joy& joy_msg){
  int num = std::min(joy_msg.buttons.size(), last_joy_msg.buttons.size());
  buttons_press.resize(num);
  buttons_release.resize(num);
  for (int i=0; i< num; i++){
    if(joy_msg.buttons[i] && !last_joy_msg.buttons[i]) buttons_press[i] = 1;
    else buttons_press[i] = 0;
    if(!joy_msg.buttons[i] && last_joy_msg.buttons[i]) buttons_release[i] = 1;
    else buttons_release[i] = 0;
  }
  last_joy_msg = joy_msg;
}

int gun_tracked_id=-1;
void object_callback(const s4_msgs::TrackedObjectArray& objects_msg){
  bool match=false;
  for(int i=0; i<objects_msg.objects.size(); i++){
    if(objects_msg.objects[i].info.id == gun_tracked_id){
      match=true;
      break;
    }
  }
  if(!match){
    if(objects_msg.objects.size()>0){
      gun_tracked_id=objects_msg.objects[0].info.id;
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_operation_test_app");
  ros::NodeHandle nh;
  ROS_INFO("START");
  Client move_client("move_action", false);
  Client turret_client("turret_action", false);
  Client select_client("select_action", false);

  ros::Subscriber joy_sub = nh.subscribe("joy", 1, joy_callback);
  ros::Subscriber object_sub = nh.subscribe("objects", 10, object_callback);

  ros::Rate loop_rate(10);
  
  while (ros::ok()){
    if(ros::Time::now() - last_joy_msg.header.stamp < ros::Duration(1.0)){
      if(std::min(std::min(last_joy_msg.buttons.size(), buttons_press.size()), buttons_release.size())>=PS3_Button_Max){
        //move
        if(move_client.isServerConnected()){
          ROS_INFO_ONCE("move server connected");
          geometry_msgs::Twist cmd_vel;
          float move_scale = 0.5;
          if(last_joy_msg.buttons[PS3_L1])move_scale=1.0;

          cmd_vel.linear.x = move_scale * last_joy_msg.axes[PS3_LV] * scale_linear_x;
          cmd_vel.angular.z = move_scale * last_joy_msg.axes[PS3_LH] * scale_angular_z;
          
          if(last_joy_msg.axes[PS3_AL2]==1 && last_joy_msg.axes[PS3_AR2]<1){
            cmd_vel.linear.y = -move_scale * scale_linear_y * std::min(1.0-last_joy_msg.axes[PS3_AR2], 1.0);
          }
          else if(last_joy_msg.axes[PS3_AR2]==1 && last_joy_msg.axes[PS3_AL2]<1){
            cmd_vel.linear.y = move_scale * scale_linear_y * std::min(1.0-last_joy_msg.axes[PS3_AL2], 1.0);
          }
          else{
            cmd_vel.linear.y = 0.0;
          }
          s4_msgs::GameAppGoal goal1;
          goal1.header.stamp=ros::Time::now();
          goal1.mode=s4_msgs::GameAppGoal::NAV_VEL;
          goal1.twist=cmd_vel;
          goal1.duration=3.0;
          move_client.sendGoal(goal1);
        }
        //turret
        if(turret_client.isServerConnected()){
          ROS_INFO_ONCE("turret server connected");
          s4_msgs::GameAppGoal goal2;
          if(buttons_press[PS3_R1]){//laser on
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
            goal2.on=true;
            turret_client.sendGoal(goal2);
          }
          else if(buttons_press[PS3_triangle]){//laser off
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
            goal2.on=false;
            turret_client.sendGoal(goal2);
          }
          else if(last_joy_msg.buttons[PS3_R1] && buttons_press[PS3_cross]){//shot
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_SHOT;
            goal2.on=true;
            turret_client.sendGoal(goal2);
          }

          static geometry_msgs::Point aim_pos;
          float aim_scale = 1.0;
          if(buttons_press[PS3_square]){//aim center
            aim_pos.y = 0.0;
            aim_pos.z = 0.0;
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_POS;
            goal2.pose.position=aim_pos;
            goal2.duration=3.0;
            turret_client.sendGoal(goal2);
          }
          else if(std::fabs(last_joy_msg.axes[PS3_RH]) + std::fabs(last_joy_msg.axes[PS3_RV]) > 0.01){
            float aim_scale = 1.0;
            if(last_joy_msg.buttons[PS3_R1])aim_scale = 0.3;
            static geometry_msgs::Twist aim_vel;
            aim_vel.linear.y = aim_scale * 0.8 * last_joy_msg.axes[PS3_RH];
            aim_vel.linear.z = aim_scale * 0.2 * -last_joy_msg.axes[PS3_RV];

            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_VEL;
            goal2.twist = aim_vel;
            goal2.duration=0.1;
            turret_client.sendGoal(goal2);
          }
          else if(buttons_press[PS3_circle]){
            goal2.header.stamp=ros::Time::now();
            //goal2.mode=s4_msgs::GameAppGoal::FC_OBJECT;
            goal2.mode=s4_msgs::GameAppGoal::FC_FOCUS;
            goal2.duration=3.0;
            goal2.info.category="red_can";
            goal2.info.id=gun_tracked_id;
            turret_client.sendGoal(goal2);
          }
        }
        if(select_client.isServerConnected()){
          s4_msgs::GameAppGoal goal2;
          if(buttons_press[PS3_Up]){//focus
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::SL_CHANGE;
            goal2.duration=1.0;
            select_client.sendGoal(goal2);
          }
        }
      }
    }
    //buttons reset
    for (int i=0; i< (int)buttons_press.size(); i++){
      buttons_press[i] = 0;
    }
    for (int i=0; i< (int)buttons_release.size(); i++){
      buttons_release[i] = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}