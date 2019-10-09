#ifndef OBSTACLE_DISPLAY_H
#define OBSTACLE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <s4_msgs/TrackedRayArray.h>
#include <s4_msgs/TrackedObjectArray.h>
#endif

namespace Ogre{
class SceneNode;
}

namespace rviz{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace s4_plugins{

class ObjectsDisplay: public rviz::MessageFilterDisplay<s4_msgs::TrackedObjectArray>{
Q_OBJECT
public:
  ObjectsDisplay();
  virtual ~ObjectsDisplay();

  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateAlpha();

private:
  void clearVis();
  void processMessage(const s4_msgs::TrackedObjectArray::ConstPtr& msg);
  geometry_msgs::Vector3 getDirection(const geometry_msgs::Quaternion& quat);
  Ogre::SceneNode* frame_node_;
  rviz::FloatProperty* alpha_property_;

  std::vector<rviz::Shape *>vis_red_can_cylinder_;
  std::vector<rviz::Shape *>vis_alvar_marker_cylinder_;
  std::vector<rviz::Arrow *>vis_alvar_marker_arrow_;

};

} // namespace s4_detection

#endif // OBSTACLE_DISPLAY_H