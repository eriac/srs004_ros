#include <s4_msgs/gameAppAction.h>
#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/JointState.h>

typedef actionlib::SimpleActionServer<s4_msgs::gameAppAction> Server;

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_gun_turret_server");
  ros::NodeHandle nh;
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joints", 10);

  Server server(nh, "turret_action", false);
  server.start();

  ros::Rate loop_rate(10);
  s4_msgs::gameAppGoalConstPtr goal;
  while (ros::ok()){
    if(server.isNewGoalAvailable()){
      goal=server.acceptNewGoal();
    }

    if(server.isActive()){
      static int train_joint=0;
      static int elevation_joint=0;
      train_joint+=goal->twist.angular.z;
      elevation_joint+=goal->twist.angular.y;

      ROS_INFO("twist (%f, %f)", goal->twist.angular.z, goal->twist.angular.y);
      {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp=ros::Time::now();
        joint_msg.name.resize(2);
        joint_msg.name[0]=std::string("train");
        joint_msg.name[1]=std::string("elevation");
        joint_msg.position.resize(2);
        joint_msg.position[0]=(float)train_joint/50.0;
        joint_msg.position[1]=(float)elevation_joint/50.0;
        joint_pub.publish(joint_msg);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}