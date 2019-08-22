// OSS
#include <math.h>
#include <sstream>
#include <string>
// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/JointControllerState.h>
#include <tf/transform_datatypes.h>

class TryOmniConverter {
public:
  TryOmniConverter() : nh_(), pnh_("~"), state0_sub_(nh_, "wheel0/state", 1), state1_sub_(nh_, "wheel1/state", 1), state2_sub_(nh_, "wheel2/state", 1), states_sync_(StatesSyncPolicy(5), state0_sub_, state1_sub_, state2_sub_) {
    pnh_.getParam("wheel_base", wheel_base_);
    pnh_.getParam("wheel_radius", wheel_radius_);
    
    wheel_base_ = 0.100;
    wheel_radius_ = 0.02;

    wheel_normal[0] = M_PI / 3 + M_PI / 2;
    wheel_normal[1] = M_PI + M_PI / 2;
    wheel_normal[2] = 5 * M_PI / 3 + M_PI / 2;

    wheel0_pub_ = nh_.advertise<std_msgs::Float64>("wheel0/command", 2);
    wheel1_pub_ = nh_.advertise<std_msgs::Float64>("wheel1/command", 2);
    wheel2_pub_ = nh_.advertise<std_msgs::Float64>("wheel2/command", 2);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 2);

    twist_sub_ = nh_.subscribe("cmd_vel", 1, &TryOmniConverter::twistCallback, this);
    states_sync_.registerCallback(&TryOmniConverter::statesSyncCallback, this);
    last_pose_.orientation.w = 1.0;
  }
  geometry_msgs::Twist wheel_invert(float *in){
    geometry_msgs::Twist out;
    float lv = 1.0 / wheel_radius_;
    float av = wheel_base_/ wheel_radius_;
    float r3 = sqrt(3);
    out.linear.x  = +(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2];
    out.linear.y  = -(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2];
    out.angular.z = -(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2];
    return out;
  }

  geometry_msgs::Pose updatePose(geometry_msgs::Pose last_pose, geometry_msgs::Twist speed, float dt){
    geometry_msgs::Pose next_pose;

    tf::Quaternion quat_tmp;
      double roll, pitch, last_yaw=0;
      //get yaw
    quaternionMsgToTF(last_pose.orientation, quat_tmp);
    tf::Matrix3x3(quat_tmp).getRPY(roll, pitch, last_yaw); 
    //update
    next_pose.position.x = last_pose.position.x + (cos(last_yaw+speed.angular.z * dt/2) * speed.linear.x - sin(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
    next_pose.position.y = last_pose.position.y + (sin(last_yaw+speed.angular.z * dt/2) * speed.linear.x + cos(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
    float next_yaw = last_yaw + speed.angular.z * dt;
    //sey yaw
    quat_tmp=tf::createQuaternionFromRPY(0,0,next_yaw);
      quaternionTFToMsg(quat_tmp, next_pose.orientation);
    return next_pose;
  }


  typedef message_filters::sync_policies::ApproximateTime<control_msgs::JointControllerState, control_msgs::JointControllerState, control_msgs::JointControllerState> StatesSyncPolicy;
  
  void twistCallback(const geometry_msgs::Twist &twist_msg);

  void statesSyncCallback(const control_msgs::JointControllerState &state0_msg,
                   const control_msgs::JointControllerState &state1_msg,
                   const control_msgs::JointControllerState &state2_msg) {

    ros::Time ref_stamp = state0_msg.header.stamp;
    ros::Duration diff1 = state1_msg.header.stamp - ref_stamp;
    ros::Duration diff2 = state2_msg.header.stamp - ref_stamp;

    ros::Time ros_time_now = ref_stamp + (diff1 + diff2) * (1.0 / 3.0);
    float in[3]; 
    in[0] = state0_msg.process_value;
    in[1] = state1_msg.process_value;
    in[2] = state2_msg.process_value;
    geometry_msgs::Twist input_twist = wheel_invert(in);

    if(last_stamp_ != ros::Time(0)){
      double dt = (ros_time_now - last_stamp_).toSec();
      last_pose_ = updatePose(last_pose_, input_twist, dt);
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "odom";
      odom_msg.header.stamp = ros_time_now;
      odom_msg.twist.twist = input_twist;
      odom_msg.pose.pose = last_pose_;
      odom_pub_.publish(odom_msg);
    }
    last_stamp_ = ros_time_now;
  }


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher wheel0_pub_;
  ros::Publisher wheel1_pub_;
  ros::Publisher wheel2_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber twist_sub_;

  message_filters::Subscriber<control_msgs::JointControllerState> state0_sub_;
  message_filters::Subscriber<control_msgs::JointControllerState> state1_sub_;
  message_filters::Subscriber<control_msgs::JointControllerState> state2_sub_;
  message_filters::Synchronizer<StatesSyncPolicy> states_sync_;
  geometry_msgs::Pose last_pose_;
  ros::Time last_stamp_;

  float wheel_base_;
  float wheel_radius_;

  float wheel_normal[3];

  void calculation(float *out, float *in) {
    for (int i = 0; i < 3; i++) {
      out[i] = -(cos(wheel_normal[i]) * in[0] + sin(wheel_normal[i]) * in[1]) /
              wheel_radius_;
      out[i] += in[2] * wheel_base_ / wheel_radius_;
    }
  }

};




void TryOmniConverter::twistCallback(const geometry_msgs::Twist &twist_msg) {
  float in[3] = {0};
  float out[3] = {0};
  in[0] = twist_msg.linear.x;
  in[1] = twist_msg.linear.y;
  in[2] = twist_msg.angular.z;
  calculation(out, in);
  std_msgs::Float64 data[3];
  data[0].data = out[0];
  data[1].data = out[1];
  data[2].data = out[2];
  wheel0_pub_.publish(data[0]);
  wheel1_pub_.publish(data[1]);
  wheel2_pub_.publish(data[2]);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "s4_omni_twist");
  TryOmniConverter try_omni_converter;

  ros::spin();
  return 0;
}
