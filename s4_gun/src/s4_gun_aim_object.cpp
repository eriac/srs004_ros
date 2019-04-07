#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <s4_msgs/TrackedInfo.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <string>

s4_msgs::TrackedObjectArray last_objects;
void object_callback(const s4_msgs::TrackedObjectArray& objects_msg) {
  last_objects = objects_msg;
}

ros::Publisher command_pub;
std::string gun_standard_link = "standard";
void focus_callback(const s4_msgs::TrackedInfo info_msg) {
  static tf::TransformListener ln;
  printf("focus %s %i\n", info_msg.category.c_str(), info_msg.id);
  int index = -1;
  for (int i = 0; i < last_objects.objects.size(); i++) {
    printf("object[%i] %s %i\n", i,
           last_objects.objects[i].info.category.c_str(),
           last_objects.objects[i].info.id);
    if (last_objects.objects[i].info.category == info_msg.category) {
      if (last_objects.objects[i].info.id == info_msg.id) {
        index = i;
        break;
      }
    }
  }
  printf("recv match:%i\n", index);
  if (index >= 0) {
    try {
      geometry_msgs::PoseStamped source_pose;
      source_pose.header.frame_id = last_objects.header.frame_id;
      source_pose.header.stamp = ros::Time(0);
      source_pose.pose.position = last_objects.objects[index].center;
      source_pose.pose.orientation.w = 1.0;

      geometry_msgs::PoseStamped target_pose;
      std::string target_frame = gun_standard_link;
      ln.waitForTransform(source_pose.header.frame_id, target_frame,
                          ros::Time(0), ros::Duration(1.0));
      ln.transformPose(target_frame, source_pose, target_pose);

      if (target_pose.pose.position.x > 0.1) {
        geometry_msgs::Point pub_point;
        pub_point.x = 1.0;
        pub_point.y = target_pose.pose.position.y / target_pose.pose.position.x;
        pub_point.z = target_pose.pose.position.z / target_pose.pose.position.x;
        ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f", pub_point.x, pub_point.y, pub_point.z);
        command_pub.publish(pub_point);
      }
    } catch (...) {
      ROS_ERROR_DELAYED_THROTTLE(5.0, "tf error %s -> %s",
                                 last_objects.header.frame_id.c_str(),
                                 gun_standard_link.c_str());
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "s4_gun_aim_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // publish
  command_pub = nh.advertise<geometry_msgs::Point>("command", 10);

  // Subscribe
  ros::Subscriber focus_sub = nh.subscribe("focus", 10, focus_callback);
  ros::Subscriber object_sub = nh.subscribe("objects", 10, object_callback);

  // rosparam
  pnh.getParam("gun_standard_link", gun_standard_link);

  ros::spin();
  return 0;
}
