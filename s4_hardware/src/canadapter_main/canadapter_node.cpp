#include "s4_hardware/usb_adapter.h"
#include "s4_hardware/msg_converter.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>
// SRS004
#include <s4_hardware/canlink_plugins_base.h>

class CANAdapter : public USBSerial{
 public:
  CANAdapter();
  bool addPlugin(std::string name, std::string type, std::string channel, int id);
  bool outputCode(s4_msgs::CANCode code);

  bool process(std::deque<unsigned char>& buffer);
  void timerCallback(const ros::TimerEvent& event);
  bool exact(std::string data);
  ros::Timer interval_timer_;

  pluginlib::ClassLoader<s4_hardware::CANLinkPluginsBase> plugins_loader_;
  std::vector<boost::shared_ptr<s4_hardware::CANLinkPluginsBase>> plugin_list_;
};

CANAdapter::CANAdapter(): USBSerial{}, plugins_loader_("s4_hardware", "s4_hardware::CANLinkPluginsBase"){
  double hz = 10.0;
  pnh_.getParam("hz", hz);
  interval_timer_ = nh_.createTimer(ros::Duration(1.0/hz), &CANAdapter::timerCallback, this);
  std::string config = "";
  pnh_.getParam("config", config);

  if(config != ""){
    YAML::Node conf = YAML::LoadFile(config);
    for(int i = 0 ; i <(int)conf["plugins"].size(); i++){
      std::string name = conf["plugins"][i]["name"].as<std::string>();
      std::string type = conf["plugins"][i]["type"].as<std::string>();
      std::string channel = conf["plugins"][i]["channel"].as<std::string>();
      int id = conf["plugins"][i]["id"].as<int>();      
      addPlugin(name, type, channel, id);
    }
  }
  else{
    ROS_INFO("empty");
  }

}

bool CANAdapter::addPlugin(std::string name, std::string type, std::string channel, int id){
  try{
    boost::shared_ptr<s4_hardware::CANLinkPluginsBase> test0 = plugins_loader_.createInstance(type);
    boost::function<bool(s4_msgs::CANCode)> func= boost::bind(&CANAdapter::outputCode, this, _1);
    test0->initialize(nh_, pnh_, name, channel, id, func);
    plugin_list_.push_back(test0);
  }
  catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
}

bool CANAdapter::outputCode(s4_msgs::CANCode code){
  //ROS_WARN("OUTPUT");

  s4_msgs::SerialCode s_code;
  std::string out0;
  
  cancode_to_serialcode(&s_code, code);
  serialcode_to_serial(&out0, s_code);

  std::vector<unsigned char> activate_data;
  activate_data.resize((int)out0.size());
  for(int i=0;i<out0.size();i++)activate_data[i]=out0[i];
  output(activate_data);
}


bool CANAdapter::process(std::deque<unsigned char>& buffer){
  //ROS_WARN("INPUT");
  std::string s0(buffer.begin(), buffer.end());
  buffer.clear();

	static std::string serial_buffer="";
	serial_buffer+=s0;
	//printf("do encode\n");
	for(;;){
		int f1=serial_buffer.find(";");
		if(f1>=0){
			//printf("find;\n");
			int f2=serial_buffer.rfind("#",f1);
			if(f2>=0){
				//printf("find#\n");
				std::string outstring=serial_buffer.substr(f2+1,f1-f2-1);
        exact(outstring);
			}
			serial_buffer=serial_buffer.substr(f1+1,serial_buffer.size()-f1-1);
			continue;
		}
		else break;
	
	}
}

void CANAdapter::timerCallback(const ros::TimerEvent& event){
  for(int i=0;i<plugin_list_.size();i++){
    plugin_list_[i]->sync();
  }


  // std::string out0("#CANLINK.S-ID.1-COM.1-REMOTE;");
  // std::vector<unsigned char> activate_data;
  // activate_data.resize((int)out0.size());
  // for(int i=0;i<out0.size();i++)activate_data[i]=out0[i];
  // output(activate_data);
}

bool CANAdapter::exact(std::string data){
  //extract
  //printf("code: %s\n", data.c_str());

  s4_msgs::SerialCode s_code;
  s4_msgs::CANCode c_code;

	serial_to_serialcode(&s_code, data);
  serialcode_to_cancode(&c_code, s_code);

  for(int i=0;i<plugin_list_.size();i++){
    plugin_list_[i]->inputCode(c_code);
  }

  // printf("c: %s, id: %i, com: %i, ", c_code.channel.c_str(), c_code.id, c_code.com);
  // if(c_code.remote)printf("remote: true\n");
  // else printf("remote: false\n");
  // printf("[%i] ", c_code.length);
  // for(int i=0;i<8;i++)printf("%i, ", c_code.data[i]);
  // printf("\n");
}


int main(int argc, char **argv){
  ros::init(argc, argv, "usb_test");
  CANAdapter usb_serial_test{};

  // pluginlib::ClassLoader<s4_hardware::CANLinkPluginsBase> plugins_loader("s4_hardware", "s4_hardware::CANLinkPluginsBase");

  // std::string plugin_name = "s4_hardware/TestPlugin";
  // try{
  //   boost::shared_ptr<s4_hardware::CANLinkPluginsBase> test0 = plugins_loader.createInstance(plugin_name);
  //   ROS_INFO("SUCCESS PLUGIN");
  // }
  // catch(pluginlib::PluginlibException& ex)
  // {
  //   ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  // }

  //usb_serial_test.addPlugin("s4_hardware/MasterPlugin", "S", 1);
  //usb_serial_test.addPlugin("s4_hardware/WheelPlugin", "A", 1);
  //usb_serial_test.addPlugin("s4_hardware/TestPlugin", "S", 1);

  ros::spin();
}
