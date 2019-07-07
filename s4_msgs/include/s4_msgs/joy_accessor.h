#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <s4_msgs/Joy.h>
#include <s4_msgs/TrackedInfo.h>
#include <s4_msgs/TrackedObjectArray.h>
#include <s4_sensor/objects_accessor.h>

#include <map>

namespace s4_msgs{
class JoyAccesor{
public:
  JoyAccesor();
  ~JoyAccesor() = default;
  void UpdateJoy(const s4_msgs::Joy& joy) ;
  s4_msgs::Joy GetCurrent(void);
  s4_msgs::Joy GetPress(void);
  s4_msgs::Joy GetRelease(void);
  void ResetChange(void);

  s4_msgs::JoySide updatePress(s4_msgs::JoySide last_press, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input);
  s4_msgs::JoySide updateRelease(s4_msgs::JoySide last_Release, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input);

  s4_msgs::Joy last_input_;
  s4_msgs::Joy press_;
  s4_msgs::Joy release_;  
};

JoyAccesor::JoyAccesor(){
}

void JoyAccesor::UpdateJoy(const s4_msgs::Joy& joy){
  press_ = updatePress(press_, joy, last_input_);
  release_ = updateRelease(release_, joy, last_input_);
  last_input_ = joy;
}

s4_msgs::Joy JoyAccesor::GetCurrent(void){
  return last_input;
}

s4_msgs::Joy JoyAccesor::GetPress(void){
  return press_;
}

s4_msgs::Joy JoyAccesor::GetRelease(void){
  return release_;
}

void JoyAccesor::ResetChange(void){
  s4_msgs::Joy zero_joy;
  press_ = zero_joy;
  release_ = zero_joy;
}

s4_msgs::JoySide JoyAccesor::updatePress(s4_msgs::JoySide last_press, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input){
  s4_msgs::JoySide output;
  output.B1 = (press.B1 > 0 || (current.B1 > 0 && last.B1 == 0)) ? 1.0 : 0.0;
  output.B2 = (press.B2 > 0 || (current.B2 > 0 && last.B2 == 0)) ? 1.0 : 0.0;
  output.B3 = (press.B3 > 0 || (current.B3 > 0 && last.B3 == 0)) ? 1.0 : 0.0;
  output.CU = (press.CU > 0 || (current.CU > 0 && last.CU == 0)) ? 1.0 : 0.0;
  output.CD = (press.CD > 0 || (current.CD > 0 && last.CD == 0)) ? 1.0 : 0.0;
  output.CL = (press.CL > 0 || (current.CL > 0 && last.CL == 0)) ? 1.0 : 0.0;
  output.CR = (press.CR > 0 || (current.CR > 0 && last.CR == 0)) ? 1.0 : 0.0;
  return output;
}

s4_msgs::JoySide JoyAccesor::updateRelease(s4_msgs::JoySide last_Release, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input){
  s4_msgs::JoySide output;
  output.B1 = (press.B1 > 0 || (current.B1 == 0 && last.B1 > 0)) ? 1.0 : 0.0;
  output.B2 = (press.B2 > 0 || (current.B2 == 0 && last.B2 > 0)) ? 1.0 : 0.0;
  output.B3 = (press.B3 > 0 || (current.B3 == 0 && last.B3 > 0)) ? 1.0 : 0.0;
  output.CU = (press.CU > 0 || (current.CU == 0 && last.CU > 0)) ? 1.0 : 0.0;
  output.CD = (press.CD > 0 || (current.CD == 0 && last.CD > 0)) ? 1.0 : 0.0;
  output.CL = (press.CL > 0 || (current.CL == 0 && last.CL > 0)) ? 1.0 : 0.0;
  output.CR = (press.CR > 0 || (current.CR == 0 && last.CR > 0)) ? 1.0 : 0.0;
  return output;
}

}// s4_msgs
