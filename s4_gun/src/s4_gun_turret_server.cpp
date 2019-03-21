#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <s4_msgs/GameAppAction.h>

typedef actionlib::SimpleActionServer<s4_msgs::GameAppAction> Server;

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_gun_turret_server");
  ros::NodeHandle nh;
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("command", 10);
  ros::Publisher laser_pub = nh.advertise<std_msgs::Bool>("laser", 10);
  ros::Publisher shot_pub = nh.advertise<std_msgs::Int32>("shot", 10);

  Server server(nh, "turret_action", false);
  server.start();

  ros::Rate loop_rate(10);
  s4_msgs::GameAppGoalConstPtr goal;
  while (ros::ok()){
    if(server.isNewGoalAvailable()){
      goal=server.acceptNewGoal();
    }

    if(server.isActive()){
      if(goal->mode == s4_msgs::GameAppGoal::FC_LASER){
        std_msgs::Bool bool_msg;
        bool_msg.data = goal->on;
        laser_pub.publish(bool_msg);
      }
      else if(goal->mode == s4_msgs::GameAppGoal::FC_SHOT){
        std_msgs::Int32 int_msg;
        int_msg.data = 3;
        shot_pub.publish(int_msg);
      }
      else if(goal->mode == s4_msgs::GameAppGoal::FC_POS){        
        ROS_INFO("point (%f, %f, %f)", goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
        geometry_msgs::Point point_msg;
        point_msg.x = goal->pose.position.x;
        point_msg.y = goal->pose.position.y;
        point_msg.z = goal->pose.position.z;
        point_pub.publish(point_msg);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}