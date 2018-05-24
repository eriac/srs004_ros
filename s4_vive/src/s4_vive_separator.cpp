#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <s4_vive/vive_general.h>

void set_frame(std::string name, float *pos, float *dir){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(  tf::Vector3(   pos[0], pos[1], pos[2]) );
	transform.setRotation(tf::Quaternion(dir[0], dir[1], dir[2], dir[3]));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));	
}

std::string controller0_name="";
std::string controller1_name="";
void vive_callback(const s4_vive::vive_general& vive_msg){
    static std::string c0_name="";
    static std::string c1_name="";
    //chane process
    if(vive_msg.buttons[2]==1){
        if(c0_name=="")                                c0_name=vive_msg.index;
        else if(c1_name=="" && c0_name!=vive_msg.index)c1_name=vive_msg.index;
    }

    float pos[3]={0.0,0.0,0.0};
    float dir[4]={0.0,0.0,0.0,1.0};
    pos[0]=vive_msg.position.x;
    pos[1]=vive_msg.position.y;
    pos[2]=vive_msg.position.z;
    dir[0]=vive_msg.orientation.x;
    dir[1]=vive_msg.orientation.y;
    dir[2]=vive_msg.orientation.z;
    dir[3]=vive_msg.orientation.w;
    if(vive_msg.index==c0_name){
        set_frame(controller0_name, pos, dir);
    }
    else if(vive_msg.index==c1_name){
        set_frame(controller1_name, pos, dir);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "s4_vive_separetor");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //subscriber
	ros::Subscriber joy_sub   = n.subscribe("general", 10, vive_callback); 

    //rosparam
	pn.getParam("controller0_name", controller0_name);
	pn.getParam("controller1_name", controller1_name);

    ros::Rate loop_rate(10);

    while (ros::ok()){
	    ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}
