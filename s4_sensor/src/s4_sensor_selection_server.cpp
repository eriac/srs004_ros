#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedInfo.h>
#include <s4_msgs/TrackedObjectArray.h>

s4_msgs::TrackedObjectArray last_objects;
void objects_callback(const s4_msgs::TrackedObjectArray& objects_msg){
  last_objects = objects_msg;
}

typedef actionlib::SimpleActionServer<s4_msgs::GameAppAction> Server;

int main(int argc, char** argv) {
  ros::init(argc, argv, "s4_sensor_selection_server");
  ros::NodeHandle nh;
  ros::Publisher aim_pub = nh.advertise<s4_msgs::TrackedInfo>("focus", 10);
  ros::Subscriber objects_sub = nh.subscribe("objects", 1, objects_callback);

  Server server(nh, "select_action", false);
  server.start();

  ros::Rate loop_rate(10);
  s4_msgs::GameAppGoalConstPtr goal;
  while (ros::ok()) {
    if (server.isNewGoalAvailable()) {
      goal = server.acceptNewGoal();
    }

    if (server.isActive()) {
      ros::Time end = goal->header.stamp + ros::Duration(goal->duration);
      if (end < ros::Time::now()) {
        server.setSucceeded();
      }

      if (goal->mode == s4_msgs::GameAppGoal::SL_CHANGE) {
        static s4_msgs::TrackedInfo tracking;
        if(last_objects.objects.size() > 0){
          int index = -1;
          for(int i=0; i<last_objects.objects.size(); i++){
            s4_msgs::TrackedInfo item = last_objects.objects[i].info;
            if(item.category == tracking.category && item.id == tracking.id){
              index = i;
              break;
            }
          }
          if(index == -1){
            tracking = last_objects.objects[0].info;
            aim_pub.publish(tracking);
          }
          else{
            index = (index + 1) % last_objects.objects.size();
            tracking = last_objects.objects[index].info;
            aim_pub.publish(tracking);
          }
        }
        server.setSucceeded();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}