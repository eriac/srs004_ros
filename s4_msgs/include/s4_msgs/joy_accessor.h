#include <ros/ros.h>
#include <s4_msgs/Joy.h>

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
  press_.right = updatePress(press_.right, joy.right, last_input_.right);
  press_.left = updatePress(press_.left, joy.left, last_input_.left);
  //release_ = updateRelease(release_, joy, last_input_);
  last_input_ = joy;
}

s4_msgs::Joy JoyAccesor::GetCurrent(void){
  return last_input_;
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
  output.B1 = (last_press.B1 > 0 || (current_input.B1 > 0 && last_input.B1 == 0)) ? 1.0 : 0.0;
  output.B2 = (last_press.B2 > 0 || (current_input.B2 > 0 && last_input.B2 == 0)) ? 1.0 : 0.0;
  output.B3 = (last_press.B3 > 0 || (current_input.B3 > 0 && last_input.B3 == 0)) ? 1.0 : 0.0;
  output.CU = (last_press.CU > 0 || (current_input.CU > 0 && last_input.CU == 0)) ? 1.0 : 0.0;
  output.CD = (last_press.CD > 0 || (current_input.CD > 0 && last_input.CD == 0)) ? 1.0 : 0.0;
  output.CL = (last_press.CL > 0 || (current_input.CL > 0 && last_input.CL == 0)) ? 1.0 : 0.0;
  output.CR = (last_press.CR > 0 || (current_input.CR > 0 && last_input.CR == 0)) ? 1.0 : 0.0;
  return output;
}

s4_msgs::JoySide JoyAccesor::updateRelease(s4_msgs::JoySide last_release, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input){
  s4_msgs::JoySide output;
  output.B1 = (last_release.B1 > 0 || (current_input.B1 == 0 && last_input.B1 > 0)) ? 1.0 : 0.0;
  output.B2 = (last_release.B2 > 0 || (current_input.B2 == 0 && last_input.B2 > 0)) ? 1.0 : 0.0;
  output.B3 = (last_release.B3 > 0 || (current_input.B3 == 0 && last_input.B3 > 0)) ? 1.0 : 0.0;
  output.CU = (last_release.CU > 0 || (current_input.CU == 0 && last_input.CU > 0)) ? 1.0 : 0.0;
  output.CD = (last_release.CD > 0 || (current_input.CD == 0 && last_input.CD > 0)) ? 1.0 : 0.0;
  output.CL = (last_release.CL > 0 || (current_input.CL == 0 && last_input.CL > 0)) ? 1.0 : 0.0;
  output.CR = (last_release.CR > 0 || (current_input.CR == 0 && last_input.CR > 0)) ? 1.0 : 0.0;
  return output;
}

}// s4_msgs
