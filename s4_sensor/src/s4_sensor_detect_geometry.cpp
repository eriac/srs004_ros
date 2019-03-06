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

class FrameDrawer{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  ros::Subscriber sub2_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  jsk_recognition_msgs::ObjectArray last_objects_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("output/image_raw", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    sub2_ = nh_.subscribe("objects", 1, &FrameDrawer::objectCb, this);
  }

  void objectCb(const jsk_recognition_msgs::ObjectArray object_msg){
    last_objects_=object_msg;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    static tf::TransformListener tflistener;

    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    for(int i=0;i<last_objects_.objects.size();i++){
      geometry_msgs::PoseStamped source_pose;
      source_pose.header.frame_id=last_objects_.header.frame_id;
      source_pose.pose.position.x=last_objects_.objects[i].dimensions.x;
      source_pose.pose.position.y=last_objects_.objects[i].dimensions.y;
      source_pose.pose.position.z=last_objects_.objects[i].dimensions.z;
      source_pose.pose.orientation.w=1.0;
      geometry_msgs::PoseStamped target_pose;
      try{
        tflistener.waitForTransform(cam_model_.tfFrame(), source_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tflistener.transformPose(cam_model_.tfFrame(),ros::Time(0),source_pose,source_pose.header.frame_id,target_pose);
      }
      catch(...){
        ROS_INFO("tf error");
      }

      if(0.1<target_pose.pose.position.z && target_pose.pose.position.z<10){
        cv::Point3d pt_cv1(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        cv::Point2d uv1 = cam_model_.project3dToPixel(pt_cv1);
        cv::circle(image, cv::Point(uv1.x,uv1.y), 30, CV_RGB(255,0,0), 5);
        cv:putText(image, last_objects_.objects[i].name.c_str(), cv::Point(uv1.x-50, uv1.y-40), cv::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255,0,0), 3);

        //printf("name: %s\n", last_objects_.objects[i].name.c_str());
        //printf("origin: x:%f, y:%f, z:%f\n", last_objects_.objects[i].dimensions.x, last_objects_.objects[i].dimensions.y, last_objects_.objects[i].dimensions.z);
        //printf("trans : x:%f, y:%f, z:%f\n", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        //printf("image : x:%f, y:%f\n", uv1.x, uv1.y);
      }
    }
    pub_.publish(input_bridge->toImageMsg());
  }
};

ros::Publisher markers_pub;
ros::Publisher objects_pub;
sensor_msgs::CameraInfo info_msg_;

visualization_msgs::Marker line_delete(void){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/default_robot/sensor0/head_camera_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.lifetime = ros::Duration();
  return marker;
}

visualization_msgs::Marker line_vis(cv::Point3d pt3, int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/default_robot/sensor0/head_camera_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.scale.x = 0.02;
  marker.scale.y = 0.04;
  marker.scale.z = 0.05;

  marker.points.resize(2);
  marker.points[0].x=0;
  marker.points[0].y=0;
  marker.points[0].z=0;
  marker.points[1].x=pt3.x;
  marker.points[1].y=pt3.y;
  marker.points[1].z=pt3.z;
    
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  return marker;
}

void rectsCallback(const s4_msgs::TrackedRectArray& rects_msg){
  s4_msgs::TrackedObjectArray objects;
  objects.header=rects_msg.header;
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info_msg_);
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(line_delete());
  for(int i=0; i<(int)rects_msg.rects.size(); i++){
    printf("[%i] x:%i, y:%i\n", 
      i,
      rects_msg.rects[i].x+rects_msg.rects[i].width/2,
      rects_msg.rects[i].y+rects_msg.rects[i].height/2);
    cv::Point2d uv;
    uv.x=rects_msg.rects[i].x+rects_msg.rects[i].width/2;
    uv.y=rects_msg.rects[i].y+rects_msg.rects[i].height/2;
    cv::Point3d pt_cv1=cam_model.projectPixelTo3dRay(uv);
    printf("ray x:%f, y:%f z:%f\n", pt_cv1.x, pt_cv1.y, pt_cv1.z);
 
    tf::Transform tf_pose;
    tf_pose.setOrigin(tf::Vector3(pt_cv1.x, pt_cv1.y, pt_cv1.z));
    tf_pose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
    
    tf::Transform tf_rotate;
    tf_rotate.setOrigin(tf::Vector3(0, 0, 0));
    tf_rotate.setRotation(tf::createQuaternionFromRPY(0.06, 0.02, 0.0));
    tf::Transform converted=tf_rotate*tf_pose;

    cv::Point3d pt_cv2;
    pt_cv2.x=converted.getOrigin().x();
    pt_cv2.y=converted.getOrigin().y();
    pt_cv2.z=converted.getOrigin().z();
    printf("ray x:%f, y:%f z:%f\n", pt_cv2.x, pt_cv2.y, pt_cv2.z);

    markers.markers.push_back(line_vis(pt_cv2, i));
  
    s4_msgs::TrackedObject object;
    object.rect=rects_msg.rects[i];
    object.center.x=pt_cv2.x;
    object.center.y=pt_cv2.y;
    object.center.z=pt_cv2.z;
    objects.objects.push_back(object);
  }
  markers_pub.publish(markers);
  objects_pub.publish(objects);
}

void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg){
  info_msg_=*info_msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_display_objects");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  double offset_roll;
  double offset_pitch;
  double offset_yaw;
  
  
  //pnh.getParam();
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
  objects_pub =  nh.advertise<s4_msgs::TrackedObjectArray>("objects", 1);
  ros::Subscriber sub1 = nh.subscribe("tracking_rects", 10, rectsCallback);
  ros::Subscriber sub2 = nh.subscribe("camera_info", 10, infoCallback);

  //std::vector<std::string> frame_ids(argv + 1, argv + argc);
  //FrameDrawer drawer(frame_ids);
  ros::spin();
}