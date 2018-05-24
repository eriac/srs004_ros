#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <s4_vive/vive_general.h>

#include <string.h>
#include "simple_udp.h"

simple_udp udp0;
std::string udp_address="0.0.0.0";
int udp_port=4001;

void set_frame(std::string name, float *pos, float *dir){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(  tf::Vector3(   pos[0], pos[1], pos[2]) );
	transform.setRotation(tf::Quaternion(dir[0], dir[1], dir[2], dir[3]));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));	
}

std::vector<std::string> split(const std::string &str, char sep){
    std::vector<std::string> v;
    std::stringstream ss(str);
    std::string buffer;
    while( std::getline(ss, buffer, sep) ) {
        v.push_back(buffer);
    }
    return v;
}

std::string controller0_name="";
std::string controller1_name="";

int main(int argc, char **argv){
  ros::init(argc, argv, "s4_vive_recieve");
  ros::NodeHandle n;
	ros::NodeHandle pn("~");

  //rosparam
	pn.getParam("udp_address",  udp_address);
	pn.getParam("udp_port",     udp_port);
	pn.getParam("controller0_name", controller0_name);
	pn.getParam("controller1_name", controller1_name);

  //publisher
  ros::Publisher data_pub   = n.advertise<s4_vive::vive_general>("general", 1);

  ros::Rate loop_rate(10);

  udp0.udp_init(udp_address,udp_port);
  udp0.udp_bind();

  while (ros::ok()){
	  std::string rdata=udp0.udp_recv_nb();
    if(rdata.length()>0){
	    ROS_INFO("recv:%s", rdata.c_str());
      std::vector<std::string>process1=split(rdata,',');
      float pos[3]={0.0,0.0,0.0};
      float dir[4]={0.0,0.0,0.0,1.0};
      int buttons[4]={0,0,0,0};
      pos[0]=atof(process1[2].c_str());
      pos[1]=atof(process1[3].c_str());
      pos[2]=atof(process1[4].c_str());
      dir[0]=atof(process1[5].c_str());
      dir[1]=atof(process1[6].c_str());
      dir[2]=atof(process1[7].c_str());
      dir[3]=atof(process1[8].c_str());
      buttons[0]=atoi(process1[ 9].c_str());
      buttons[1]=atoi(process1[10].c_str());
      buttons[2]=atoi(process1[11].c_str());
      buttons[3]=atoi(process1[12].c_str());

      s4_vive::vive_general general0;
      general0.type=process1[0];
      general0.index=process1[1];
      general0.position.x=pos[0];
      general0.position.y=pos[1];
      general0.position.z=pos[2];
      general0.orientation.x=dir[0];
      general0.orientation.y=dir[1];
      general0.orientation.z=dir[2];
      general0.orientation.w=dir[3];
      general0.buttons[0]=buttons[0];
      general0.buttons[1]=buttons[1];
      general0.buttons[2]=buttons[2];
      general0.buttons[3]=buttons[3];
      data_pub.publish(general0);
    }
  }
  return 0;
}
