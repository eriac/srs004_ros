#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedInfo.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

s4_msgs::TrackedInfo last_focus;
void focus_callback(const s4_msgs::TrackedInfo& focus_msg){
  last_focus = focus_msg;
}

geometry_msgs::Point last_point;
void command_callback(const geometry_msgs::Point& point_msg){
  last_point = point_msg;
}

typedef actionlib::SimpleActionServer<s4_msgs::GameAppAction> Server;

int main(int argc, char** argv) {
  ros::init(argc, argv, "s4_gun_turret_server");
  ros::NodeHandle nh;
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("command", 10);
  ros::Publisher laser_pub = nh.advertise<std_msgs::Bool>("laser", 10);
  ros::Publisher shot_pub = nh.advertise<std_msgs::Int32>("shot", 10);
  ros::Publisher focus_pub = nh.advertise<s4_msgs::TrackedInfo>("focus", 10);

  ros::Subscriber point_sub = nh.subscribe("command", 10, command_callback);
  ros::Subscriber focus_sub = nh.subscribe("input_focus", 10, focus_callback);

  Server server(nh, "turret_action", false);
  server.start();

  ros::Rate loop_rate(10);
  s4_msgs::GameAppGoalConstPtr goal;

  float aim_lim_y = 0.62;
  float aim_lim_z = 0.16;//0.12

  static geometry_msgs::Point aim_pos;
  while (ros::ok()) {
    if (server.isNewGoalAvailable()) {
      goal = server.acceptNewGoal();
    }

    if (server.isActive()) {
      ros::Time end = goal->header.stamp + ros::Duration(goal->duration);
      if (end < ros::Time::now()) {
        server.setSucceeded();
      }

      if (goal->mode == s4_msgs::GameAppGoal::FC_LASER) {
        std_msgs::Bool bool_msg;
        bool_msg.data = goal->on;
        laser_pub.publish(bool_msg);
      }
      else if (goal->mode == s4_msgs::GameAppGoal::FC_SHOT) {
        std_msgs::Int32 int_msg;
        int_msg.data = 3;
        shot_pub.publish(int_msg);
      }
      else if (goal->mode == s4_msgs::GameAppGoal::FC_VEL) {
        float dt=0.1;
        aim_pos.x=1.0;
        float tmp_y = last_point.y + goal->twist.linear.y * dt;
        float tmp_z = last_point.z + goal->twist.linear.z * dt;
        aim_pos.y = std::max(-aim_lim_y, std::min(aim_lim_y, tmp_y));
        aim_pos.z = std::max(-aim_lim_z, std::min(aim_lim_z, tmp_z));
        point_pub.publish(aim_pos);
      }
      else if (goal->mode == s4_msgs::GameAppGoal::FC_POS) {
        ROS_INFO("point (%f, %f, %f)", goal->pose.position.x,
                 goal->pose.position.y, goal->pose.position.z);
        geometry_msgs::Point point_msg;
        point_msg.x = goal->pose.position.x;
        point_msg.y = goal->pose.position.y;
        point_msg.z = goal->pose.position.z;
        point_pub.publish(point_msg);
      }
      else if (goal->mode == s4_msgs::GameAppGoal::FC_OBJECT) {
        ROS_INFO("focus_object %s, %i", goal->info.category.c_str(), goal->info.id);
        focus_pub.publish(goal->info);
      }
      else if (goal->mode == s4_msgs::GameAppGoal::FC_FOCUS) {
        ROS_INFO("focus! %s, %i", last_focus.category.c_str(), last_focus.id);
        focus_pub.publish(last_focus);
      }
      else{
        ROS_ERROR("Unknown mode: %i", goal->mode);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}