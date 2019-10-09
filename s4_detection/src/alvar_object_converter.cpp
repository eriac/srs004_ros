// OSS
#include <opencv/cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// ROS
#include <ar_track_alvar_msgs/AlvarMarkers.h>
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

struct alvarConfig
{
  int id;
  tf::Transform config_tf;
};

class AlvarObjectConverter
{
public:
  AlvarObjectConverter() : nh_(), pnh_("~")
  {
    objects_pub_ = nh_.advertise<s4_msgs::TrackedObjectArray>("objects", 1);
    alvar_sub_ = nh_.subscribe("ar_pose_marker", 10, &AlvarObjectConverter::alvarCallback, this);
    fusion_frame_ = "s4n1/odom";
    pnh_.getParam("fusion_frame", fusion_frame_);
    XmlRpc::XmlRpcValue alvar_list;
    pnh_.getParam("alvar_list", alvar_list);
    ROS_ASSERT(alvar_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    configs_.clear();
    for (int i = 0; i < alvar_list.size(); i++)
    {
      configs_.push_back(makeConfig(alvar_list[i]));
    }
  }

  void alvarCallback(const ar_track_alvar_msgs::AlvarMarkers& alvar_msg)
  {
    tf::Transform alver_basic_tf;
    alver_basic_tf.setOrigin(tf::Vector3(0, 0, 0));
    alver_basic_tf.setRotation(tf::createQuaternionFromRPY(0, M_PI / 2.0, -M_PI / 2.0));
    s4_msgs::TrackedObjectArray output_objects;
    output_objects.header.frame_id = fusion_frame_;
    output_objects.header.stamp = ros::Time::now();

    for (auto marker : alvar_msg.markers)
    {
      for (auto config : configs_)
      {
        if (marker.id == config.id)
        {
          geometry_msgs::PoseStamped source_pose;
          source_pose.header = marker.header;
          source_pose.pose = marker.pose.pose;
          geometry_msgs::PoseStamped target_pose;
          listener_.waitForTransform(source_pose.header.frame_id, fusion_frame_, ros::Time(0), ros::Duration(1.0));
          listener_.transformPose(fusion_frame_, source_pose, target_pose);
          tf::Transform marker_tf;
          tf::poseMsgToTF(target_pose.pose, marker_tf);
          tf::Transform object_tf = marker_tf * alver_basic_tf * config.config_tf;
          geometry_msgs::Pose object_pose;
          tf::poseTFToMsg(object_tf, object_pose);
          object_pose = project2d(object_pose);
          output_objects.objects.push_back(makeObject(marker.id, object_pose));
        }
      }
    }
    objects_pub_.publish(output_objects);
  }

  alvarConfig makeConfig(XmlRpc::XmlRpcValue xmlrpc)
  {
    alvarConfig output;
    ROS_ASSERT(xmlrpc["id"].valid());
    output.id = static_cast<int>(xmlrpc["id"]);
    double x = static_cast<double>(xmlrpc["x"]);
    double y = static_cast<double>(xmlrpc["y"]);
    double z = static_cast<double>(xmlrpc["z"]);
    output.config_tf.setOrigin(tf::Vector3(x, y, z));
    double roll = static_cast<double>(xmlrpc["roll"]);
    double pitch = static_cast<double>(xmlrpc["pitch"]);
    double yaw = static_cast<double>(xmlrpc["yaw"]);
    output.config_tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    return output;
  }

  geometry_msgs::Pose project2d(geometry_msgs::Pose input_msg)
  {
    geometry_msgs::Pose output_msg;
    output_msg.position.x = input_msg.position.x;
    output_msg.position.y = input_msg.position.y;
    output_msg.position.z = 0;
    
    tf::Quaternion input_rotation_tf;
    tf::quaternionMsgToTF(input_msg.orientation, input_rotation_tf);
    double roll, pitch, yaw;
    tf::Matrix3x3(input_rotation_tf).getRPY(roll, pitch, yaw);
    tf::Quaternion output_rotation_tf(tf::createQuaternionFromRPY(0, 0, yaw));
    tf::quaternionTFToMsg(output_rotation_tf, output_msg.orientation);
    return output_msg;
  }

  s4_msgs::TrackedObject makeObject(int id, geometry_msgs::Pose input_pose){
    s4_msgs::TrackedObject output;
    output.info.category = "alvar_marker";
    output.info.id = id;
    output.presence.pose = input_pose;
    return output;
  } 

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener listener_;
  ros::Publisher objects_pub_;
  ros::Subscriber alvar_sub_;
  std::string fusion_frame_;

  obstacle_detector::ObstaclesConstPtr last_obstacles_ptr_;
  std::vector<alvarConfig> configs_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_fusion");
  AlvarObjectConverter alvar_object_converter;
  ros::spin();
}