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

#include <Eigen/Core>
#include <Eigen/Geometry>

class DetectMerger
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener listener_;
  ros::Publisher objects_pub_;
  ros::Publisher markers_pub_;

  typedef message_filters::sync_policies::ApproximateTime<s4_msgs::TrackedObjectArray, obstacle_detector::Obstacles> MySyncPolicy;
  message_filters::Subscriber<s4_msgs::TrackedObjectArray> rects_sub_;
  message_filters::Subscriber<obstacle_detector::Obstacles> obstacle_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;

  std::string merge_frame_;
  double distance_max_;
  double obstacle_height_;

public:
  DetectMerger() : nh_(), pnh_("~"),
                   rects_sub_(nh_, "tracked_rays", 10),
                   obstacle_sub_(nh_, "tracked_obstacles", 20),
                   sync_(MySyncPolicy(10), rects_sub_, obstacle_sub_)
  {
    objects_pub_ = nh_.advertise<s4_msgs::TrackedObjectArray>("objects", 1);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);

    sync_.registerCallback(&DetectMerger::sync_callback, this);

    merge_frame_ = "default_robot/base_link";
    distance_max_ = 0.06;
    obstacle_height_ = 0.06;
    pnh_.getParam("merge_frame", merge_frame_);
    pnh_.getParam("distance_max", distance_max_);
    pnh_.getParam("obstacle_height", obstacle_height_);
  }
  void sync_callback(const s4_msgs::TrackedObjectArray &object_msg, const obstacle_detector::Obstacles &obstacle_msg)
  {
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(line_delete());
    geometry_msgs::Point zero_position;
    Eigen::Vector3d origin = getVector(zero_position, object_msg.header.frame_id, merge_frame_);
    s4_msgs::TrackedObjectArray merge_objects;
    merge_objects.header=obstacle_msg.header;

    for (int i = 0; i < (int)object_msg.objects.size(); i++)
    {
      Eigen::Vector3d project = getVector(object_msg.objects[i].center, object_msg.header.frame_id, merge_frame_);
      int obstacle_size = obstacle_msg.circles.size();
      std::vector<Eigen::Vector3d> points;
      points.resize(obstacle_size);
      std::vector<double> scale;
      scale.resize(obstacle_size);
      std::vector<double> dists;
      dists.resize(obstacle_size);
      for (int j = 0; j < obstacle_size; j++)
      {
        geometry_msgs::Point obstacle_position = obstacle_msg.circles[j].center;
        obstacle_position.z = obstacle_height_;
        Eigen::Vector3d point = getVector(obstacle_position, obstacle_msg.header.frame_id, merge_frame_);

        Eigen::Vector3d ray = (project - origin).normalized();
        Eigen::Vector3d ref = point - origin;
        scale[j] = ref.dot(ray);
        points[j] = origin + ray * scale[j];
        dists[j] = ref.cross(ray).norm();
      }
      if (obstacle_size > 0)
      {
        std::vector<double>::iterator iter = std::min_element(dists.begin(), dists.end());
        int index = std::distance(dists.begin(), iter);

        if (dists[index] < distance_max_)
        {
          geometry_msgs::Point position;
          position.x = points[index](0);
          position.y = points[index](1);
          position.z = points[index](2);

          Eigen::Vector3d s_size = scale[index] * getVector(object_msg.objects[i].size, object_msg.header.frame_id, merge_frame_);
          geometry_msgs::Vector3 size;
          size.x = s_size(0);
          size.y = s_size(1);
          size.z = s_size(2);
          double radius = sqrt(s_size(0) * s_size(0) + s_size(1) * s_size(1)) / 2;

          s4_msgs::TrackedObject merge_object;
          merge_object.info = object_msg.objects[i].info;
          merge_object.rect = object_msg.objects[i].rect;
          merge_object.center = position;
          merge_object.velocity = obstacle_msg.circles[index].velocity;
          merge_object.size = size;
          merge_objects.objects.push_back(merge_object);

          markers.markers.push_back(cylinder_vis(position, radius, s_size(2), merge_frame_, i));          
        }
      }
    }
    markers_pub_.publish(markers);
    objects_pub_.publish(merge_objects);
  }

  visualization_msgs::Marker line_delete(void)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "";
    marker.header.stamp = ros::Time::now();
    marker.ns = "merge_shapes";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.lifetime = ros::Duration();
    return marker;
  }

  Eigen::Vector3d getVector(geometry_msgs::Point position, std::string source_frame, std::string target_frame)
  {
    Eigen::Vector3d vector;
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id = source_frame;
    source_pose.pose.orientation.w = 1.0;
    source_pose.pose.position = position;
    geometry_msgs::PoseStamped target_pose;
    listener_.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
    listener_.transformPose(target_frame, source_pose, target_pose);
    vector(0) = target_pose.pose.position.x;
    vector(1) = target_pose.pose.position.y;
    vector(2) = target_pose.pose.position.z;
    return vector;
  }
  Eigen::Vector3d getVector(geometry_msgs::Vector3 size, std::string source_frame, std::string target_frame)
  {
    geometry_msgs::Point position;
    Eigen::Vector3d p0 = getVector(position, source_frame, target_frame);
    position.x = size.x;
    position.y = size.y;
    position.z = size.z;
    Eigen::Vector3d p1 = getVector(position, source_frame, target_frame);
    return p1-p0;
  }

  visualization_msgs::Marker cylinder_vis(geometry_msgs::Point position, float radius, float height, std::string frame_id, int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "merge_shapes";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = height;

    marker.pose.position = position;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    return marker;
  }

  visualization_msgs::Marker point_vis(geometry_msgs::Point position, float radius, std::string frame_id, int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "merge_shapes_point";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;

    marker.pose.position = position;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    return marker;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "s4_sensor_detect_merger");
  DetectMerger detect_merger;
  ros::spin();
}
