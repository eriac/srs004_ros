#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <s4_msgs/Joy.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <algorithm>

#include <s4_msgs/objects_accessor.h>

class TestApp{
public:
  TestApp();
  void objectsCallback(const s4_msgs::TrackedObjectArray& objects_msg);
  void timerCallback(const s4_msgs::TrackedObjectArray& objects_msg);
  void timerCallback(const ros::TimerEvent&);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber objects_sub_;
  ros::Timer interval_timer;

  s4_msgs::ObjectsAccessor objects_accessor_;
  
};

TestApp::TestApp() : nh_(), pnh_(), objects_accessor_() {
  objects_sub_ = nh_.subscribe("objects", 1, &TestApp::objectsCallback, this);
  interval_timer = nh_.createTimer(ros::Duration(0.1), &TestApp::timerCallback, this);

}

void TestApp::objectsCallback(const s4_msgs::TrackedObjectArray& objects_msg){
  objects_accessor_.UpdateObjects(objects_msg);
}

void TestApp::timerCallback(const ros::TimerEvent&){
  // s4_msgs::TrackedInfo info;
  // bool ret = objects_accessor_.GetFocus(info);
  // if(ret){
  //   ROS_INFO("category: %s, id: %i", info.category.c_str(), info.id);
  // }
  // else{
  //   ROS_INFO("nothing");
  // }


  if(objects_accessor_.FocusExist()){
    s4_msgs::TrackedInfo o_info;
    objects_accessor_.GetFocus(o_info);
    ROS_INFO("info: %s %i", o_info.category.c_str(), o_info.id);  
    return;
  }

  std::vector<std::string> categories;
  bool ret = objects_accessor_.GetCategories(categories);
  if(ret){
    ROS_INFO("category: %s", categories[0].c_str());
    objects_accessor_.SetCategory(categories[0]);
    geometry_msgs::Point refferece_point;
    objects_accessor_.SetNearest(refferece_point);

    s4_msgs::TrackedInfo o_info;
    bool ret2 = objects_accessor_.GetFocus(o_info);
    if(ret2){
      ROS_INFO("info: %s %i", o_info.category.c_str(), o_info.id);  
    }
  }
  else{
    ROS_INFO("category: nothing");
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "s4_operation_test2");
  TestApp app0;
  ros::spin();
  return 0;
}