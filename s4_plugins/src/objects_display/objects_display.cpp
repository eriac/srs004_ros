#include "objects_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <s4_msgs/TrackedObjectArray.h>

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

void ObjectsDisplay::reset(){
  // clear
  for(auto& cylinder : vis_cylinder_){
    delete cylinder;
  }
  vis_cylinder_.clear();
  
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
  for(auto& cylinder : vis_cylinder_){
    delete cylinder;
  }
  vis_cylinder_.clear();

  frame_node_ = scene_node_->createChildSceneNode();
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);

  for(auto object : msg->objects){
    if(object.info.category == "redcan"){
      rviz::Shape *cylinder;
      cylinder = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_);

      auto& p = object.presence.point;
      Ogre::Vector3 shape_pos(p.x, p.y, p.z);
      cylinder->setPosition(shape_pos); 

      float r = object.presence.radius.data;
      float l = object.presence.height.data;
      Ogre::Vector3 shape_scale(r, l, r);
      cylinder->setScale(shape_scale); 

      Ogre::Quaternion  shape_q(0.7, 0.7, 0, 0); // (w, x, y, z)
      cylinder->setOrientation(shape_q);   
      float alpha = alpha_property_->getFloat();
      cylinder->setColor(1, 0, 0, alpha);

      vis_cylinder_.push_back(cylinder);

    }
  }
}

} // namespace s4_detection

PLUGINLIB_EXPORT_CLASS(s4_plugins::ObjectsDisplay, rviz::Display )