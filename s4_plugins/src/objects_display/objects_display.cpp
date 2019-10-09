#include "objects_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <cmath>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <tf/transform_datatypes.h>

namespace s4_plugins{

ObjectsDisplay::ObjectsDisplay(){
  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateAlpha() ));
}

void ObjectsDisplay::onInitialize(){
  MFDClass::onInitialize(); // MFDClass := MessageFilterDisplay<message type>
}

ObjectsDisplay::~ObjectsDisplay(){
}

void ObjectsDisplay::clearVis(){
  for(auto& vis : vis_red_can_cylinder_)delete vis;
  vis_red_can_cylinder_.clear();

  for(auto& vis : vis_alvar_marker_cylinder_)delete vis;
  vis_alvar_marker_cylinder_.clear();

  for(auto& vis : vis_alvar_marker_arrow_)delete vis;
  vis_alvar_marker_arrow_.clear();
}

void ObjectsDisplay::reset(){
  clearVis();
  MFDClass::reset();
}

void ObjectsDisplay::updateAlpha(){
}


void ObjectsDisplay::processMessage(const s4_msgs::TrackedObjectArray::ConstPtr& msg){
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)){
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // clear
  clearVis();

  frame_node_ = scene_node_->createChildSceneNode();
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);

  for(auto object : msg->objects){
    if(object.info.category == "red_can"){
      rviz::Shape *cylinder;
      cylinder = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_);

      auto& p = object.presence.pose.position;
      Ogre::Vector3 shape_pos(p.x, p.y, p.z);
      cylinder->setPosition(shape_pos); 

      float r = object.presence.radius.data;
      float l = object.presence.height.data;
      Ogre::Vector3 shape_scale(r * 2.0, l, r * 2.0);
      cylinder->setScale(shape_scale); 

      Ogre::Quaternion  shape_q(0.7, 0.7, 0, 0); // (w, x, y, z)
      cylinder->setOrientation(shape_q);   
      float alpha = alpha_property_->getFloat();
      cylinder->setColor(1, 0, 0, alpha);

      vis_red_can_cylinder_.push_back(cylinder);
    }
    else if(object.info.category == "alvar_marker"){
      geometry_msgs::Vector3 v = getDirection(object.presence.pose.orientation);
      auto& p = object.presence.pose.position;
      float l = 0.04;
      float r = 0.2;
      float alpha = alpha_property_->getFloat();
      
      // cylinder
      rviz::Shape *cylinder;
      cylinder = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_);
      Ogre::Vector3 shape_pos(p.x, p.y, p.z - l * 0.5);
      cylinder->setPosition(shape_pos); 
      Ogre::Vector3 shape_scale(r * 2.0, l, r * 2.0);
      cylinder->setScale(shape_scale); 
      Ogre::Quaternion  shape_q(0.7, 0.7, 0, 0); // (w, x, y, z)
      cylinder->setOrientation(shape_q);
      cylinder->setColor(0, 0, 1, alpha);
      vis_alvar_marker_cylinder_.push_back(cylinder);

      // arrow
      rviz::Arrow *arrow;
      arrow = new rviz::Arrow(scene_manager_, frame_node_);
      Ogre::Vector3 arrow_pos(p.x - v.x * r, p.y - v.y * r, p.z - v.z * r);
      arrow->setPosition(arrow_pos);
      arrow->setColor(1.0, 0.0, 0.0, alpha);
      Ogre::Vector3 arrow_dir(v.x * r * 2.0, v.y * r * 2.0, v.z * r * 2.0);
      float arrow_length = arrow_dir.length() * 0.77;
      Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
      arrow->setScale(arrow_scale);
      arrow->setDirection(arrow_dir);
      vis_alvar_marker_arrow_.push_back(arrow);
    }
  }
}

geometry_msgs::Vector3 ObjectsDisplay::getDirection(const geometry_msgs::Quaternion& quat){
  geometry_msgs::Vector3 output_vector;
  tf::Quaternion rotation_tf;
  tf::quaternionMsgToTF(quat, rotation_tf);
  double roll, pitch, yaw;
  tf::Matrix3x3(rotation_tf).getRPY(roll, pitch, yaw);
  output_vector.x = cos(yaw);
  output_vector.y = sin(yaw);
  output_vector.z = 0;
  return output_vector;
}

} // namespace s4_detection

PLUGINLIB_EXPORT_CLASS(s4_plugins::ObjectsDisplay, rviz::Display )