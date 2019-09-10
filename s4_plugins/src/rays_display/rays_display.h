#ifndef POINT_DISPLAY_H
#define POINT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <s4_msgs/TrackedRectArray.h>
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

class RaysDisplay: public rviz::MessageFilterDisplay<s4_msgs::TrackedRectArray>{
Q_OBJECT
public:
  RaysDisplay();
  virtual ~RaysDisplay();

  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateLength();

private:
  void processMessage( const s4_msgs::TrackedRectArray::ConstPtr& msg );
  Ogre::SceneNode* frame_node_;
  std::vector<rviz::Arrow *>vis_arrows_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* length_property_;
};

} // namespace s4_detection

#endif // POINT_DISPLAY_H