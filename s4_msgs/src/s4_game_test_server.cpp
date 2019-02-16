#include <s4_msgs/gameAppAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<s4_msgs::gameAppAction> Server;

int main(int argc, char** argv){
  ros::init(argc, argv, "test_server");
  ros::NodeHandle n;
  Server server(n, "test_app", false);
  server.start();

  ros::Rate loop_rate(10);
  s4_msgs::gameAppGoalConstPtr goal;
  while (ros::ok()){
    if(server.isNewGoalAvailable()){
      goal=server.acceptNewGoal();
    }

    if(server.isActive()){
      ros::Time end=goal->header.stamp+ros::Duration(goal->duration);
      if(end<ros::Time::now()){
        ROS_INFO("twist %f",(float)0);
        server.setSucceeded();
      }
      else{
        ROS_INFO("twist (%f, %f)", goal->twist.linear.x, goal->twist.linear.y);
      }
    }  
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}