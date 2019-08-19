#ifndef _S4_HARDWARE_CANLINK_PLUGINS_BASE_
#define _S4_HARDWARE_CANLINK_PLUGINS_BASE_

#include <s4_msgs/CANCode.h>

namespace s4_hardware{
class CANLinkPluginsBase{
public:
  virtual void sync(void);
  virtual void input(s4_msgs::CANCode code);
};
}
#endif