#include "rays_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <s4_msgs/TrackedRectArray.h>

namespace s4_plugins{

RaysDisplay::RaysDisplay(){
  color_property_ = new rviz::ColorProperty( "Color", QColor( 200, 50, 50 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  length_property_ = new rviz::FloatProperty( "length", 1.0,
                                             "length to draw arrows",
                                             this, SLOT( updateLength() ));
}

void RaysDisplay::onInitialize(){
  // frame_node_ = scene_node_->createChildSceneNode();
  // vis_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_));
  // float alpha = alpha_property_->getFloat();
  // Ogre::ColourValue color = color_property_->getOgreColor();
  // vis_arrow_->setColor(color.r, color.g, color.b, alpha);
  // Ogre::Vector3 arrow_scale(0, 0, 0);
  // vis_arrow_->setScale(arrow_scale);
  MFDClass::onInitialize(); // MFDClass := MessageFilterDisplay<message type>
}

RaysDisplay::~RaysDisplay(){
}

void RaysDisplay::reset(){
  MFDClass::reset();
}

void RaysDisplay::updateColorAndAlpha(){
  // float alpha = alpha_property_->getFloat();
  // Ogre::ColourValue color = color_property_->getOgreColor();
  // vis_arrow_->setColor(color.r, color.g, color.b, alpha);
}

void RaysDisplay::updateLength(){
}

void RaysDisplay::processMessage(const s4_msgs::TrackedRectArray::ConstPtr& msg){
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)){
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // clear
  for(auto& arrow : vis_arrows_){
    delete arrow;
  }
  vis_arrows_.clear();

  frame_node_ = scene_node_->createChildSceneNode();
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);

  for(auto ray : msg->rects){
    rviz::Arrow *arrow;
    arrow = new rviz::Arrow(scene_manager_, frame_node_);
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    arrow->setColor(color.r, color.g, color.b, alpha);

    Ogre::Vector3 arrow_dir(ray.ray.x, ray.ray.y, ray.ray.z);
    float arrow_length = arrow_dir.length() * 0.77 * length_property_->getFloat();
    Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
    arrow->setScale(arrow_scale);
    arrow->setDirection(arrow_dir);

    vis_arrows_.push_back(arrow);
  }
}

} // namespace s4_detection

PLUGINLIB_EXPORT_CLASS(s4_plugins::RaysDisplay,rviz::Display )