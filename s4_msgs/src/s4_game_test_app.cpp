#include <s4_msgs/gameAppAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<s4_msgs::gameAppAction> Client;

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_msgs_test_app");
  Client client("test_app", true); // true -> don't need ros::spin()
  client.waitForServer();
  s4_msgs::gameAppGoal goal;
  //goal.twist.x=1.0;

  goal.mode=3;
  goal.header.stamp=ros::Time::now();
  goal.twist.linear.x=0.5;
  goal.duration=3.0;
  client.sendGoal(goal);
    
  for(int i=0;i<10;i++){
    ROS_INFO("goal");
    ros::Duration(1.0).sleep();
    ROS_INFO("Current State: %s", client.getState().toString().c_str());
  }
  return 0;
}