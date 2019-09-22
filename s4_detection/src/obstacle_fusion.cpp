// OSS
#include <opencv/cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/ObjectArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <tf/transform_broadcaster.h>
#include <obstacle_detector/Obstacles.h>
// SRS004
#include <s4_msgs/TrackedObjectArray.h>
#include <s4_msgs/TrackedRectArray.h>

class ObstacleFusion{
public:
  ObstacleFusion() : nh_(), pnh_("~") {
    objects_pub_ = nh_.advertise<s4_msgs::TrackedObjectArray>("objects", 1);

    rects_sub_ = nh_.subscribe("tracked_rays", 10, &ObstacleFusion::raysCallback, this);
    obstacle_sub_ = nh_.subscribe("tracked_obstacles", 10, &ObstacleFusion::obstacleCallback, this);


    fusion_frame_ = "default_robot/base_link";
    distance_max_ = 0.06;
    obstacle_height_ = 0.06;
    pnh_.getParam("fusion_frame", fusion_frame_);
    pnh_.getParam("distance_max", distance_max_);
    pnh_.getParam("obstacle_height", obstacle_height_);

    // temp
    fusion_frame_ = "s4n1/odom";
    obstacle_height_ = 0.05;
  }

  void raysCallback(const s4_msgs::TrackedRectArray& rays_msg){
    if(!last_obstacles_ptr_ || last_obstacles_ptr_->circles.empty()){
      ROS_WARN_DELAYED_THROTTLE(5.0, "obstacle not available");
      return;
    }

    int ray_size = rays_msg.rects.size();
    int obstacle_size;
    if(last_obstacles_ptr_)obstacle_size = last_obstacles_ptr_->circles.size();
    //asart obstacle_frame_id is odom

    s4_msgs::TrackedObjectArray output_objects;
    output_objects.header.frame_id = fusion_frame_;
    output_objects.header.stamp = ros::Time::now();    
    
    Eigen::Vector3d origin_point = convertPoint(fusion_frame_, rays_msg.header.frame_id);
    //ROS_INFO("origin: %f %f %f", origin_point[0], origin_point[1], origin_point[2]);
    for(int i = 0; i < (int)rays_msg.rects.size(); i++){
      Eigen::Vector3d normal_point = convertPoint(fusion_frame_, rays_msg.header.frame_id, rays_msg.rects[i].ray);
      //ROS_INFO("ray: %f %f %f", normal_point[0], normal_point[1], normal_point[2]);

      Eigen::Vector3d fusion_point;
      int index = getNearestObstacle(origin_point, normal_point, fusion_point);
      if(index >= 0){
        s4_msgs::TrackedObject object = makeObject(index, fusion_point, rays_msg.rects[i]);
        output_objects.objects.push_back(object);
      }
    }
    objects_pub_.publish(output_objects);
  }

  void obstacleCallback(const obstacle_detector::ObstaclesConstPtr& obstacle_msg){
    if(obstacle_msg->header.frame_id != fusion_frame_){
      ROS_WARN_THROTTLE(5.0, "input frame_id: %s is not fusion_frame: %s", obstacle_msg->header.frame_id.c_str(), fusion_frame_.c_str());
    }
    last_obstacles_ptr_ = obstacle_msg;
  }

  Eigen::Vector3d convertPoint(std::string fusion_frame, std::string camera_frame){
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id = camera_frame;
    source_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped target_pose;
    listener_.waitForTransform(source_pose.header.frame_id, fusion_frame, ros::Time(0), ros::Duration(1.0));
    listener_.transformPose(fusion_frame, source_pose, target_pose);

    Eigen::Vector3d output;
    output[0] = target_pose.pose.position.x;
    output[1] = target_pose.pose.position.y;
    output[2] = target_pose.pose.position.z;
    return output;
  }

  Eigen::Vector3d convertPoint(std::string fusion_frame, std::string camera_frame, geometry_msgs::Point normal_point){
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id = camera_frame;
    source_pose.pose.orientation.w = 1.0;
    source_pose.pose.position = normal_point;

    geometry_msgs::PoseStamped target_pose;
    listener_.waitForTransform(source_pose.header.frame_id, fusion_frame, ros::Time(0), ros::Duration(1.0));
    listener_.transformPose(fusion_frame, source_pose, target_pose);

    Eigen::Vector3d output;
    output[0] = target_pose.pose.position.x;
    output[1] = target_pose.pose.position.y;
    output[2] = target_pose.pose.position.z;
    return output;
  }

  int getNearestObstacle(Eigen::Vector3d origin_point, Eigen::Vector3d normal_point, Eigen::Vector3d& fusion_point){
    int obstacle_size = last_obstacles_ptr_->circles.size();
    std::vector<double> lengthes;
    lengthes.resize(obstacle_size);
    std::vector<Eigen::Vector3d> points;
    points.resize(obstacle_size);
    std::vector<double> distances;
    distances.resize(obstacle_size);
    for(int i = 0; i < obstacle_size; i++){
      Eigen::Vector3d obstacle;
      obstacle[0] = last_obstacles_ptr_->circles[i].center.x;
      obstacle[1] = last_obstacles_ptr_->circles[i].center.y;
      obstacle[2] = obstacle_height_;

      Eigen::Vector3d ray = (normal_point - origin_point).normalized();
      Eigen::Vector3d ref = obstacle - origin_point;
      // ROS_INFO("obs[%i]ray %f %f %f", i, ray[0], ray[1], ray[2]);
      // ROS_INFO("obs[%i]ref %f %f %f", i, ref[0], ref[1], ref[2]);

      lengthes[i] = ref.dot(ray);
      points[i] = origin_point + ray * lengthes[i];
      distances[i] = ref.cross(ray).norm();

      // ROS_INFO("obs[%i] length:%f, distance:%f", i, lengthes[i], distances[i]);
    }
    std::vector<double>::iterator iter = std::min_element(distances.begin(), distances.end());
    int index = std::distance(distances.begin(), iter);
    //ROS_INFO("obs length:%f, distance:%f", lengthes[index], distances[index]);
    if(distances[index] < distance_max_ && lengthes[index] > 0){
      fusion_point = points[index];
      return index;
    }
    else return -1;
  }

  s4_msgs::TrackedObject makeObject(int index, Eigen::Vector3d fusion_point, s4_msgs::TrackedRect rect){
    s4_msgs::TrackedObject output;
    output.info = rect.info;
    output.rect = rect.rect;
    output.presence.point.x = fusion_point[0];
    output.presence.point.y = fusion_point[1];
    output.presence.point.z = fusion_point[2];
    output.presence.velocity = last_obstacles_ptr_->circles[index].velocity;
    output.presence.height.data = 1.0;
    output.presence.radius.data = 1.0;
    return output;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener listener_;
  ros::Publisher objects_pub_;

  ros::Subscriber rects_sub_;
  ros::Subscriber obstacle_sub_;

  std::string fusion_frame_;
  double distance_max_;
  double obstacle_height_;

  obstacle_detector::ObstaclesConstPtr last_obstacles_ptr_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_fusion");
  ObstacleFusion obstacle_fusion;
  ros::spin();
}