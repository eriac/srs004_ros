#ifndef OBSTACLE_DISPLAY_H
#define OBSTACLE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
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
  void processMessage(const s4_msgs::TrackedObjectArray::ConstPtr& msg);
  Ogre::SceneNode* frame_node_;

  std::vector<rviz::Shape *>vis_cylinder_;
  rviz::FloatProperty* alpha_property_;
};

} // namespace s4_detection

#endif // OBSTACLE_DISPLAY_H