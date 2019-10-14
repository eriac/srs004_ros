// OSS
#include <opencv/cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/ObjectArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <obstacle_detector/Obstacles.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// SRS004
#include <s4_msgs/TrackedObjectArray.h>
#include <s4_msgs/TrackedRayArray.h>

void callback(const s4_msgs::TrackedObjectArray::ConstPtr& msg, int* k)
{
  ROS_INFO("recv");
}

class ObjectMerger
{
public:
  ObjectMerger() : nh_(), pnh_("~")
  {
    objects_pub_ = nh_.advertise<s4_msgs::TrackedObjectArray>("objects", 1);

    fusion_frame_ = "s4n1/odom";
    pnh_.getParam("fusion_frame", fusion_frame_);

    input_num_ = 1;
    pnh_.getParam("input_num", input_num_);

    if(1 <= input_num_)objects0_subs_ = nh_.subscribe("input0", 1, &ObjectMerger::object0_callback, this);
    if(2 <= input_num_)objects1_subs_ = nh_.subscribe("input1", 1, &ObjectMerger::object1_callback, this);
    if(3 <= input_num_)objects2_subs_ = nh_.subscribe("input2", 1, &ObjectMerger::object2_callback, this);
    if(4 <= input_num_)objects3_subs_ = nh_.subscribe("input3", 1, &ObjectMerger::object3_callback, this);

    double hz = 10.0;
    pnh_.getParam("hz", hz);
    timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &ObjectMerger::timerCallback, this);
  }

  void object0_callback(const s4_msgs::TrackedObjectArray::ConstPtr& msg_ptr)
  {
    last_object0_ptr_ = msg_ptr;
  }

  void object1_callback(const s4_msgs::TrackedObjectArray::ConstPtr& msg_ptr)
  {
    last_object1_ptr_ = msg_ptr;
  }

  void object2_callback(const s4_msgs::TrackedObjectArray::ConstPtr& msg_ptr)
  {
    last_object2_ptr_ = msg_ptr;
  }

  void object3_callback(const s4_msgs::TrackedObjectArray::ConstPtr& msg_ptr)
  {
    last_object3_ptr_ = msg_ptr;
  }

  void timerCallback(const ros::TimerEvent& e){
    s4_msgs::TrackedObjectArray output_objects;
    output_objects.header.frame_id = fusion_frame_;
    output_objects.header.stamp = ros::Time::now();

    if(last_object0_ptr_){
      for(auto object : last_object0_ptr_->objects){
        output_objects.objects.push_back(object);
      }
    }
    if(last_object1_ptr_){
      for(auto object : last_object1_ptr_->objects){
        output_objects.objects.push_back(object);
      }
    }
    objects_pub_.publish(output_objects);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher objects_pub_;
  ros::Subscriber objects0_subs_;
  ros::Subscriber objects1_subs_;
  ros::Subscriber objects2_subs_;
  ros::Subscriber objects3_subs_;
  s4_msgs::TrackedObjectArray::ConstPtr last_object0_ptr_;
  s4_msgs::TrackedObjectArray::ConstPtr last_object1_ptr_;
  s4_msgs::TrackedObjectArray::ConstPtr last_object2_ptr_;
  s4_msgs::TrackedObjectArray::ConstPtr last_object3_ptr_;
  ros::Timer timer_;

  std::string fusion_frame_;
  int input_num_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_merger");
  ObjectMerger obstacle_fusion;
  ros::spin();
}