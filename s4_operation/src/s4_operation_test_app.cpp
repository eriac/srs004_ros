#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <s4_msgs/Joy.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <algorithm>

typedef actionlib::SimpleActionClient<s4_msgs::GameAppAction> Client;

float scale_linear_x=0.3;
float scale_linear_y=0.3;
float scale_angular_z=2.5;

s4_msgs::JoySide set_press(s4_msgs::JoySide press, s4_msgs::JoySide current, s4_msgs::JoySide last){
  s4_msgs::JoySide output;
  output.B1 = (press.B1 > 0 || current.B1 > 0 &&  last.B1 == 0) ? 1.0 : 0.0;
  output.B2 = (press.B2 > 0 || current.B2 > 0 &&  last.B2 == 0) ? 1.0 : 0.0;
  output.B3 = (press.B3 > 0 || current.B3 > 0 &&  last.B3 == 0) ? 1.0 : 0.0;
  output.CU = (press.CU > 0 || current.CU > 0 &&  last.CU == 0) ? 1.0 : 0.0;
  output.CD = (press.CD > 0 || current.CD > 0 &&  last.CD == 0) ? 1.0 : 0.0;
  output.CL = (press.CL > 0 || current.CL > 0 &&  last.CL == 0) ? 1.0 : 0.0;
  output.CR = (press.CR > 0 || current.CR > 0 &&  last.CR == 0) ? 1.0 : 0.0;
  return output;
}

s4_msgs::Joy buttons_press;
s4_msgs::Joy buttons_release;
s4_msgs::Joy last_joy_msg;
void joy_callback(const s4_msgs::Joy& joy_msg){
  //buttons_press
  buttons_press.left = set_press(buttons_press.left, joy_msg.left, last_joy_msg.left);
  buttons_press.right = set_press(buttons_press.right, joy_msg.right, last_joy_msg.right);
  //buttons_release
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

  ros::Subscriber joy_sub = nh.subscribe("standard_joy", 1, joy_callback);
  ros::Subscriber object_sub = nh.subscribe("objects", 10, object_callback);

  ros::Rate loop_rate(10);
  
  while (ros::ok()){
    if(ros::Time::now() - last_joy_msg.header.stamp < ros::Duration(1.0)){
      if(true){
        //move
        if(move_client.isServerConnected()){
          ROS_INFO_ONCE("move server connected");
          geometry_msgs::Twist cmd_vel;
          float move_scale = 0.5;
          if(last_joy_msg.left.B1 > 0)move_scale = 1.0;

          cmd_vel.linear.x = move_scale * last_joy_msg.left.AV * scale_linear_x;
          cmd_vel.angular.z = move_scale * last_joy_msg.left.AH * scale_angular_z;
          if(last_joy_msg.left.B2 > 0 && last_joy_msg.right.B2 == 0){
            cmd_vel.linear.y = move_scale * scale_linear_y * last_joy_msg.left.B2;
          }
          else if(last_joy_msg.left.B2 == 0 && last_joy_msg.right.B2 > 0){
            cmd_vel.linear.y = -move_scale * scale_linear_y * last_joy_msg.right.B2;
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
          if(buttons_press.right.B1){//laser on
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
            goal2.on=true;
            turret_client.sendGoal(goal2);
          }
          else if(buttons_press.right.CU){//laser off
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_LASER;
            goal2.on=false;
            turret_client.sendGoal(goal2);
          }
          else if(last_joy_msg.right.B1 > 0 && buttons_press.right.CD){//shot
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_SHOT;
            goal2.on=true;
            turret_client.sendGoal(goal2);
          }

          static geometry_msgs::Point aim_pos;
          float aim_scale = 1.0;
          if(buttons_press.right.CL){//aim center
            aim_pos.x = 1.0;
            aim_pos.y = 0.0;
            aim_pos.z = 0.0;
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_POS;
            goal2.pose.position=aim_pos;
            goal2.duration=3.0;
            turret_client.sendGoal(goal2);
          }
          else if(std::fabs(last_joy_msg.right.AH) + std::fabs(last_joy_msg.right.AV) > 0.01){
            float aim_scale = 1.0;
            if(last_joy_msg.right.B1)aim_scale = 0.3;
            static geometry_msgs::Twist aim_vel;
            aim_vel.linear.y = aim_scale * 0.8 * last_joy_msg.right.AH;
            aim_vel.linear.z = aim_scale * 0.2 * -last_joy_msg.right.AV;

            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::FC_VEL;
            goal2.twist = aim_vel;
            goal2.duration=0.1;
            turret_client.sendGoal(goal2);
          }
          else if(buttons_press.right.CR){
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
          if(buttons_press.left.CU){//focus
            goal2.header.stamp=ros::Time::now();
            goal2.mode=s4_msgs::GameAppGoal::SL_CHANGE;
            goal2.duration=1.0;
            select_client.sendGoal(goal2);
          }
        }
      }
    }
    //buttons reset

    s4_msgs::JoySide none;
    buttons_press.left = none;
    buttons_press.right = none;
    buttons_release.left = none;
    buttons_release.right = none;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}