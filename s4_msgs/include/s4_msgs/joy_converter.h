#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <s4_msgs/Joy.h>

namespace s4_msgs{
class JoyConverter{
public:
  JoyConverter(){
    
  }

  void setDevice(std::string name){
    device_ = name;
  }

  s4_msgs::Joy convert(const sensor_msgs::Joy& input){
    if(device_ == "ps3"){
      s4_msgs::Joy output;
      output.header = input.header;
      output.left.AV = input.axes[1];
      output.left.AH = input.axes[0];
      output.left.B1 = (1 - input.axes[14]) / 2 * input.buttons[10];
      output.left.B2 = (1 - input.axes[12]) / 2 * input.buttons[8];
      output.left.B3 = input.buttons[1];
      output.left.CU = input.buttons[4];//8
      output.left.CD = input.buttons[6];//10
      output.left.CL = input.buttons[7];//11? nw
      output.left.CR = input.buttons[5];//9
      output.right.AV = input.axes[3];
      output.right.AH = input.axes[2];
      output.right.B1 = (1 - input.axes[15]) / 2 * input.buttons[11];
      output.right.B2 = (1 - input.axes[13]) / 2 * input.buttons[9];
      output.right.B3 = input.buttons[2];
      output.right.CU = (1 - input.axes[16]) / 2 * input.buttons[12];
      output.right.CD = (1 - input.axes[18]) / 2 * input.buttons[14];
      output.right.CL = (1 - input.axes[19]) / 2 * input.buttons[15];
      output.right.CR = (1 - input.axes[17]) / 2 * input.buttons[13];
      output.other.resize(2);
      output.other[0] = input.buttons[0];
      output.other[1] = input.buttons[3];
      return output;
    }
  }
  std::string device_;
};
}// s4_msgs

// {
// public:
//   JoyAccesor();
//   ~JoyAccesor() = default;
//   void UpdateJoy(const s4_msgs::Joy& joy) ;
//   s4_msgs::Joy GetCurrent(void);
//   s4_msgs::Joy GetPress(void);
//   s4_msgs::Joy GetRelease(void);
//   void ResetChange(void);

//   s4_msgs::JoySide updatePress(s4_msgs::JoySide last_press, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input);
//   s4_msgs::JoySide updateRelease(s4_msgs::JoySide last_Release, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input);

//   s4_msgs::Joy last_input_;
//   s4_msgs::Joy press_;
//   s4_msgs::Joy release_;  
// };

// JoyAccesor::JoyAccesor(){
// }

// void JoyAccesor::UpdateJoy(const s4_msgs::Joy& joy){
//   press_ = updatePress(press_, joy, last_input_);
//   release_ = updateRelease(release_, joy, last_input_);
//   last_input_ = joy;
// }

// s4_msgs::Joy JoyAccesor::GetCurrent(void){
//   return last_input;
// }

// s4_msgs::Joy JoyAccesor::GetPress(void){
//   return press_;
// }

// s4_msgs::Joy JoyAccesor::GetRelease(void){
//   return release_;
// }

// void JoyAccesor::ResetChange(void){
//   s4_msgs::Joy zero_joy;
//   press_ = zero_joy;
//   release_ = zero_joy;
// }

// s4_msgs::JoySide JoyAccesor::updatePress(s4_msgs::JoySide last_press, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input){
//   s4_msgs::JoySide output;
//   output.B1 = (press.B1 > 0 || (current.B1 > 0 && last.B1 == 0)) ? 1.0 : 0.0;
//   output.B2 = (press.B2 > 0 || (current.B2 > 0 && last.B2 == 0)) ? 1.0 : 0.0;
//   output.B3 = (press.B3 > 0 || (current.B3 > 0 && last.B3 == 0)) ? 1.0 : 0.0;
//   output.CU = (press.CU > 0 || (current.CU > 0 && last.CU == 0)) ? 1.0 : 0.0;
//   output.CD = (press.CD > 0 || (current.CD > 0 && last.CD == 0)) ? 1.0 : 0.0;
//   output.CL = (press.CL > 0 || (current.CL > 0 && last.CL == 0)) ? 1.0 : 0.0;
//   output.CR = (press.CR > 0 || (current.CR > 0 && last.CR == 0)) ? 1.0 : 0.0;
//   return output;
// }

// s4_msgs::JoySide JoyAccesor::updateRelease(s4_msgs::JoySide last_Release, s4_msgs::JoySide current_input, s4_msgs::JoySide last_input){
//   s4_msgs::JoySide output;
//   output.B1 = (press.B1 > 0 || (current.B1 == 0 && last.B1 > 0)) ? 1.0 : 0.0;
//   output.B2 = (press.B2 > 0 || (current.B2 == 0 && last.B2 > 0)) ? 1.0 : 0.0;
//   output.B3 = (press.B3 > 0 || (current.B3 == 0 && last.B3 > 0)) ? 1.0 : 0.0;
//   output.CU = (press.CU > 0 || (current.CU == 0 && last.CU > 0)) ? 1.0 : 0.0;
//   output.CD = (press.CD > 0 || (current.CD == 0 && last.CD > 0)) ? 1.0 : 0.0;
//   output.CL = (press.CL > 0 || (current.CL == 0 && last.CL > 0)) ? 1.0 : 0.0;
//   output.CR = (press.CR > 0 || (current.CR == 0 && last.CR > 0)) ? 1.0 : 0.0;
//   return output;
// }

