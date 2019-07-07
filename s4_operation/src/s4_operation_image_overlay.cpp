#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <s4_msgs/Joy.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <s4_msgs/objects_accessor.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <image_geometry/pinhole_camera_model.h>

#include <s4_msgs/TrackedObjectArray.h>


#include <string>
#include <sstream>

class ImageOverlay{
public:
  ImageOverlay();
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
                 
  void objectsCallback(const s4_msgs::TrackedObjectArray& objects_msg);
  void laserScanCallback(const sensor_msgs::LaserScan& laser_scan_msg);
  void voltageCallback(const std_msgs::Float32& float_msg);
  void aimPointCallback(const geometry_msgs::Point& point_msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  tf::TransformListener ln_;

  s4_msgs::ObjectsAccessor objects_accessor_;


  image_transport::CameraSubscriber camera_sub_;
  image_transport::Publisher camera_pub_;
  ros::Subscriber laser_scan_sub_;
  sensor_msgs::LaserScan last_laser_scan_;
  ros::Subscriber voltage_sub_;
  double last_voltage_;
  ros::Subscriber aim_point_sub_;
  geometry_msgs::PointStamped last_aim_point_;

  ros::Subscriber objects_sub_;

  bool overlayVoltage(cv::Mat& image, const double voltage);
  bool overlayLaser(cv::Mat& image, const sensor_msgs::LaserScan& laser_scan);
  bool overlayAim(cv::Mat& image, const sensor_msgs::CameraInfo& info, const geometry_msgs::PointStamped& point);
  bool overlayObjects(cv::Mat& image, const s4_msgs::ObjectsAccessor& accessor);
};

ImageOverlay::ImageOverlay() : nh_(), pnh_(), it_(nh_), ln_(), objects_accessor_() {
  objects_sub_ = nh_.subscribe("objects", 1, &ImageOverlay::objectsCallback, this);
  camera_sub_ = it_.subscribeCamera("image_rect_color", 1, &ImageOverlay::imageCallback, this);
  camera_pub_ = it_.advertise("image_overlay", 1);
  laser_scan_sub_ = nh_.subscribe("scan", 1, &ImageOverlay::laserScanCallback, this);
  voltage_sub_ = nh_.subscribe("voltage", 1, &ImageOverlay::voltageCallback, this);
  aim_point_sub_ = nh_.subscribe("aim_point", 1, &ImageOverlay::aimPointCallback, this);;
}

void ImageOverlay::objectsCallback(const s4_msgs::TrackedObjectArray& objects_msg){
  objects_accessor_.UpdateObjects(objects_msg);
  
  //SetFocus
  std::vector<std::string> categories;
  objects_accessor_.GetCategories(categories);
  if(!categories.empty()){
    objects_accessor_.SetCategory(categories[0]);
    geometry_msgs::Point refferece_point;
    objects_accessor_.SetNearest(refferece_point);
  }
}

void ImageOverlay::laserScanCallback(const sensor_msgs::LaserScan& laser_scan_msg){
  last_laser_scan_ = laser_scan_msg;
}

void ImageOverlay::voltageCallback(const std_msgs::Float32& float_msg){
  last_voltage_ = float_msg.data;
}

void ImageOverlay::aimPointCallback(const geometry_msgs::Point& point_msg){
  last_aim_point_.header.stamp = ros::Time::now();
  last_aim_point_.header.frame_id = "/s4n1/gun0/standard";
  last_aim_point_.point = point_msg;
}

void ImageOverlay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
  cv_bridge::CvImagePtr input_bridge;
  input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = input_bridge->image;

  overlayVoltage(image, last_voltage_);
  overlayLaser(image, last_laser_scan_);
  overlayAim(image, *info_msg, last_aim_point_);
  overlayObjects(image, objects_accessor_);

  sensor_msgs::ImagePtr output_image = input_bridge->toImageMsg();
  camera_pub_.publish(output_image);
}

bool ImageOverlay::overlayVoltage(cv::Mat& image, const double voltage){
  std::stringstream ss;
  std::string str = "V: ";
  ss << str <<  std::fixed << std::setprecision(1) << voltage;
  cv::putText(image, ss.str().c_str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 2.0, CV_RGB(255,0,0), 3, CV_AA);
}

bool ImageOverlay::overlayLaser(cv::Mat& image, const sensor_msgs::LaserScan& laser_scan){
  tf::StampedTransform transform; 
  std::string target_frame = "/s4n1/base_link";
  try{
    ln_.lookupTransform(target_frame, laser_scan.header.frame_id, ros::Time(0), transform);
  }
  catch(...){
    ROS_INFO("tf error [%s -> %s]", target_frame.c_str(), laser_scan.header.frame_id.c_str());
    return false;
  }

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 laser_pc2, laser_pc2_transformed;
  projector.projectLaser(laser_scan, laser_pc2);
  pcl_ros::transformPointCloud(target_frame, transform, laser_pc2, laser_pc2_transformed);
  sensor_msgs::PointCloud laser_pc;
  sensor_msgs::convertPointCloud2ToPointCloud(laser_pc2_transformed, laser_pc);

  int center_u = 640;
  int center_v = 210;
  double scale = 200; // [m] -> [px]

  int box_pu = 150;
  int box_nu = 150;
  int box_pv = 50;
  int box_nv = 200;

  cv::rectangle(image, cv::Point(center_u - box_nu, center_v - box_nv), cv::Point(center_u + box_pu, center_v + box_pv), CV_RGB(255,0,0), 3);
  cv::circle(image, cv::Point(center_u, center_v), (int)(scale * 0.12), CV_RGB(255,0,0), 3);
  for(int i = 0; i < laser_pc.points.size(); i++){
    int pu = center_u - scale * laser_pc.points[i].y;
    int pv = center_v - scale * laser_pc.points[i].x;
    if(center_u - box_nu < pu && pu < center_u + box_pu && center_v - box_nv < pv && pv < center_v + box_pv){
      cv::circle(image,cv::Point(pu, pv), 3, CV_RGB(255,0,0), -1);
    }
  }
  return true;
}

bool ImageOverlay::overlayAim(cv::Mat& image, const sensor_msgs::CameraInfo& info, const geometry_msgs::PointStamped& point_msg){
  if(point_msg.header.frame_id.empty()){
    return false;
  }
  tf::StampedTransform transform;
  try{
    ln_.lookupTransform(info.header.frame_id, point_msg.header.frame_id, ros::Time(0), transform);
  }
  catch(...){
    ROS_INFO("tf error [%s -> %s]", info.header.frame_id.c_str(), point_msg.header.frame_id.c_str());
    return false;
  }

  //ROS_INFO("tr: x:%5.2f, y:%5.2f, z:%5.2f", (double)transform.getOrigin().x(), (double)transform.getOrigin().y(), (double)transform.getOrigin().z());

  geometry_msgs::Pose pose_msg;
  pose_msg.position = point_msg.point;
  pose_msg.orientation.w = 1.0;
  tf::Transform point_tf;
  poseMsgToTF(pose_msg, point_tf);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info);

  // marker1
  auto marker1_tf = transform * (point_tf.getOrigin() * 0.5);
  cv::Point3d marker1_3d;
  marker1_3d.x = marker1_tf.x();
  marker1_3d.y = marker1_tf.y();
  marker1_3d.z = marker1_tf.z();
  cv::Point2d mark1_2d = cam_model.project3dToPixel (marker1_3d);
  cv::circle(image, mark1_2d, 30, CV_RGB(255,0,0), 2);

  // marker2
  auto marker2_tf = transform * (point_tf.getOrigin() * 1.5);
  cv::Point3d marker2_3d;
  marker2_3d.x = marker2_tf.x();
  marker2_3d.y = marker2_tf.y();
  marker2_3d.z = marker2_tf.z();
  cv::Point2d mark2_2d = cam_model.project3dToPixel (marker2_3d);
  cv::circle(image, mark2_2d, 20, CV_RGB(255,0,0), 2);

  return true;
}

bool ImageOverlay::overlayObjects(cv::Mat& image, const s4_msgs::ObjectsAccessor& accessor){
  s4_msgs::TrackedObjectArray list = accessor.getList();
  int focus_index = accessor.getFocusIndex();
  s4_msgs::TrackedInfo info;
  accessor.GetFocus(info);

  ROS_INFO("focus %s %i", info.category.c_str(), info.id);
  ROS_INFO("index %i", focus_index);

  for(int i = 0; i < (int)(list.objects.size()); i++){
    cv::Point point1, point2, point3, point4;
    point1.x = list.objects[i].rect.x;
    point1.y = list.objects[i].rect.y;
    point2.x = list.objects[i].rect.x + list.objects[i].rect.width;
    point2.y = list.objects[i].rect.y + list.objects[i].rect.height;
    point3.x = list.objects[i].rect.x;
    point3.y = list.objects[i].rect.y - 40;
    point4.x = list.objects[i].rect.x;
    point4.y = list.objects[i].rect.y - 10;
    if(i == focus_index){
      cv::rectangle(image, point1, point2, CV_RGB(255, 0, 0), 3);
    }
    else{
      cv::rectangle(image, point1, point2, CV_RGB(0, 255, 0), 3);
    }
    cv::putText(image, list.objects[i].info.category, point3, cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0), 3, CV_AA);
    std::stringstream ss;
    std::string str = "id: ";
    ss << str <<  list.objects[i].info.id;
    cv::putText(image, ss.str(), point4, cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0), 3, CV_AA);
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "s4_operation_test2");
  ImageOverlay image_overlay;
  ros::spin();
  return 0;
}