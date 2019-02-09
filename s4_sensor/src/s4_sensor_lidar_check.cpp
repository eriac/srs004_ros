#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <tf/tf.h>
#include "tf/transform_broadcaster.h"

#include <string>
#include "math.h"

ros::Publisher laser_pub;
void laser_callback(const sensor_msgs::LaserScan laser_msg){
  ros::Time ctime=ros::Time::now();
  ROS_WARN("laser: %f", laser_msg.header.stamp.toSec());
  ROS_WARN("now  : %f", ctime.toSec());
  ROS_WARN("diff  :%f", (laser_msg.header.stamp-ctime).toSec());
  ROS_WARN("---");

  sensor_msgs::LaserScan laser2_msg=laser_msg;
  laser2_msg.time_increment=0;
  //laser2_msg.scan_time=0;
  laser_pub.publish(laser2_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "s4_omni_demo");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  //publish
  laser_pub = n.advertise<sensor_msgs::LaserScan>("laser_out", 10);

  //subscriibe
  ros::Subscriber laser_sub   = n.subscribe("laser_in", 10, laser_callback);

  ros::Rate loop_rate(20);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}
