#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/ObjectArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <s4_msgs/TrackedRect.h>
#include <s4_msgs/TrackedRectArray.h>
#include <s4_msgs/TrackedObject.h>
#include <s4_msgs/TrackedObjectArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <obstacle_detector/Obstacles.h>

ros::Publisher markers_pub;

void sync_callback(const s4_msgs::TrackedObjectArray& rects_msg, const obstacle_detector::Obstacles& obstacle_msg)
{
  printf("sync\n");
  // Solve all of perception here...
}

typedef message_filters::sync_policies::ApproximateTime<s4_msgs::TrackedObjectArray, obstacle_detector::Obstacles> MySyncPolicy;

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_display_objects");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  double offset_roll;
  double offset_pitch;
  double offset_yaw;
  
  //pnh.getParam();
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
  
  message_filters::Subscriber<s4_msgs::TrackedObjectArray> rects_sub(nh, "objects", 10);
  message_filters::Subscriber<obstacle_detector::Obstacles> obstacle_sub(nh, "tracked_obstacles", 20);

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rects_sub, obstacle_sub);
  //message_filters::TimeSynchronizer<> sync(rects_sub, obstacle_sub, 5);
  //sync.registerCallback(boost::bind(&sync_callback, _1, _2));
  sync.registerCallback(&sync_callback);

  ros::spin();
  return 0;
}
