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

#include <Eigen/Core>
#include <Eigen/Geometry>

ros::Publisher markers_pub;
ros::Publisher objects_pub;
image_geometry::PinholeCameraModel cam_model_;

double offset_roll_ = 0;
double offset_pitch_ = 0;
double offset_yaw_ = 0;

visualization_msgs::Marker line_delete(void)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/default_robot/sensor0/head_camera_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.lifetime = ros::Duration();
  return marker;
}

visualization_msgs::Marker line_vis(cv::Point3d pt3, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "default_robot/sensor0/head_camera_link";
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
  marker.points[0].x = 0;
  marker.points[0].y = 0;
  marker.points[0].z = 0;
  marker.points[1].x = pt3.x;
  marker.points[1].y = pt3.y;
  marker.points[1].z = pt3.z;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  return marker;
}

Eigen::Vector3d projection(cv::Point2d uv)
{
  cv::Point3d xy = cam_model_.projectPixelTo3dRay(uv);

  tf::Transform tf_pose;
  tf_pose.setOrigin(tf::Vector3(xy.x, xy.y, xy.z));
  tf_pose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));

  tf::Transform tf_rotate;
  tf_rotate.setOrigin(tf::Vector3(0, 0, 0));
  tf_rotate.setRotation(tf::createQuaternionFromRPY(offset_roll_, offset_pitch_, offset_yaw_));
  tf::Transform converted = tf_rotate * tf_pose;

  Eigen::Vector3d ray;
  ray(0) = converted.getOrigin().x();
  ray(1) = converted.getOrigin().y();
  ray(2) = converted.getOrigin().z();
  return ray;
}

void rectsCallback(const s4_msgs::TrackedRectArray &rects_msg)
{

  s4_msgs::TrackedObjectArray objects;
  objects.header = rects_msg.header;
  //objects.header.frame_id=cam_model.tfFrame();
  objects.header.frame_id = "default_robot/sensor0/head_camera_link";
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(line_delete());
  for (int i = 0; i < (int)rects_msg.rects.size(); i++)
  {
    cv::Point2d uv_cv;
    uv_cv.x = rects_msg.rects[i].rect.x + (rects_msg.rects[i].rect.width - 1) / 2;
    uv_cv.y = rects_msg.rects[i].rect.y + (rects_msg.rects[i].rect.height - 1) / 2;
    Eigen::Vector3d center = projection(uv_cv);

    cv::Point2d uv_lt;
    uv_lt.x = rects_msg.rects[i].rect.x;
    uv_lt.y = rects_msg.rects[i].rect.y;
    Eigen::Vector3d left_top = projection(uv_lt);

    cv::Point2d uv_rb;
    uv_rb.x = rects_msg.rects[i].rect.x + rects_msg.rects[i].rect.width;
    uv_rb.y = rects_msg.rects[i].rect.y + rects_msg.rects[i].rect.height;
    Eigen::Vector3d right_bottom = projection(uv_rb);


    s4_msgs::TrackedObject object;
    object.info = rects_msg.rects[i].info;
    object.rect = rects_msg.rects[i].rect;
    object.center.x = center(0);
    object.center.y = center(1);
    object.center.z = center(2);
    object.size.x = (right_bottom - left_top)(0);
    object.size.y = (right_bottom - left_top)(1);
    object.size.z = (right_bottom - left_top)(2);
    objects.objects.push_back(object);
  }
  markers_pub.publish(markers);
  objects_pub.publish(objects);
}



void infoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  cam_model_.fromCameraInfo(*info_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_display_objects");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
  objects_pub = nh.advertise<s4_msgs::TrackedObjectArray>("objects", 1);
  ros::Subscriber sub1 = nh.subscribe("tracking_rects", 10, rectsCallback);
  ros::Subscriber sub2 = nh.subscribe("camera_info", 10, infoCallback);

  pnh.getParam("offset_roll", offset_roll_);
  pnh.getParam("offset_pitch", offset_pitch_);
  pnh.getParam("offset_yaw", offset_yaw_);

  ros::spin();
}