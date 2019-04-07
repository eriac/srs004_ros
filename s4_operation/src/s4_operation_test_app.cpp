#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedObjectArray.h>

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

typedef actionlib::SimpleActionClient<s4_msgs::GameAppAction> Client;

float scale_linear_x=0.3;
float scale_linear_y=0.3;
float scale_angular_z=2.5;

ros::Time last_joy_stamp;
sensor_msgs::Joy last_joy_msg;
void joy_callback(const sensor_msgs::Joy& joy_msg){
  last_joy_stamp = joy_msg.header.stamp;
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

  ros::Subscriber joy_sub = nh.subscribe("joy", 1, joy_callback);
  ros::Subscriber object_sub = nh.subscribe("objects", 10, object_callback);

  ros::Rate loop_rate(10);
  
  while (ros::ok()){
    if(ros::Time::now() - last_joy_stamp < ros::Duration(1.0)){
      //move
      if(move_client.isServerConnected()){
        ROS_INFO_ONCE("move server connected");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x =last_joy_msg.axes[PS3_LV]*scale_linear_x;
        cmd_vel.linear.y =last_joy_msg.axes[PS3_LH]*scale_linear_y;
        cmd_vel.angular.z=last_joy_msg.axes[PS3_RH]*scale_angular_z;

        s4_msgs::GameAppGoal goal1;
        goal1.header.stamp=ros::Time::now();
        goal1.mode=s4_msgs::GameAppGoal::NAV_VEL;
        goal1.twist=cmd_vel;
        goal1.duration=3.0;
        move_client.sendGoal(goal1);
      }
      //turret
      if(turret_client.isServerConnected()){//laser on
        ROS_INFO_ONCE("turret server connected");
        s4_msgs::GameAppGoal goal2;
        if(last_joy_msg.buttons[PS3_triangle]){
          goal2.header.stamp=ros::Time::now();
          goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
          goal2.on=true;
          turret_client.sendGoal(goal2);
        }
        else if(last_joy_msg.buttons[PS3_square]){//laser off
          goal2.header.stamp=ros::Time::now();
          goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
          goal2.on=false;
          turret_client.sendGoal(goal2);
        }
        else if(last_joy_msg.buttons[PS3_R2] && last_joy_msg.buttons[PS3_R1]){//shot
          goal2.header.stamp=ros::Time::now();
          goal2.mode=s4_msgs::GameAppGoal::FC_SHOT;
          goal2.on=true;
          turret_client.sendGoal(goal2);
        }

        static geometry_msgs::Point aim_pos;
        bool isAimMove=last_joy_msg.buttons[PS3_Right]||last_joy_msg.buttons[PS3_Left]||last_joy_msg.buttons[PS3_Up]||last_joy_msg.buttons[PS3_Down];
        if(isAimMove){
          float dt=0.1;
          aim_pos.x=1.0;
          if(last_joy_msg.buttons[PS3_Right])aim_pos.y += -0.4 * dt;
          if(last_joy_msg.buttons[PS3_Left])aim_pos.y += 0.4 * dt;
          if(last_joy_msg.buttons[PS3_Up])aim_pos.z += -0.05 * dt;
          if(last_joy_msg.buttons[PS3_Down])aim_pos.z += 0.05 * dt;
          goal2.header.stamp=ros::Time::now();
          goal2.mode=s4_msgs::GameAppGoal::FC_POS;
          goal2.pose.position=aim_pos;
          goal2.duration=3.0;
          turret_client.sendGoal(goal2);
        }
        else if(last_joy_msg.buttons[PS3_circle]){
          goal2.header.stamp=ros::Time::now();
          goal2.mode=s4_msgs::GameAppGoal::FC_OBJECT;
          goal2.duration=3.0;
          goal2.info.category="red_can";
          goal2.info.id=gun_tracked_id;
          turret_client.sendGoal(goal2);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}