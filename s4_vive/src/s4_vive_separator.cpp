#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <s4_vive/vive_general.h>

ros::Publisher controller0_pub;
ros::Publisher controller1_pub;
ros::Publisher tracker0_pub;
void vive_callback(const s4_vive::vive_general& vive_msg){
    static std::string c0_name="";
    static std::string c1_name="";
    static std::string t0_name="";

    //register
    if(vive_msg.type=="C"){
        if(vive_msg.buttons[2]==1){
            if(c0_name=="")                                c0_name=vive_msg.index;
            else if(c1_name=="" && c0_name!=vive_msg.index)c1_name=vive_msg.index;
        }
    }
    else if(vive_msg.type=="T"){
        if(t0_name==""){
            t0_name=vive_msg.index;
            printf("register %s\n",t0_name.c_str());
        }
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
        controller0_pub.publish(vive_msg);
    }
    else if(vive_msg.index==c1_name){
        controller1_pub.publish(vive_msg);
    }
    else if(vive_msg.index==t0_name){
        tracker0_pub.publish(vive_msg);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "s4_vive_separetor");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

	//publisher
	controller0_pub = n.advertise<s4_vive::vive_general>("controller0/data", 1);
	controller1_pub = n.advertise<s4_vive::vive_general>("controller1/data", 1);
	tracker0_pub    = n.advertise<s4_vive::vive_general>("tracker0/data", 1);

    //subscriber
	ros::Subscriber joy_sub   = n.subscribe("general", 10, vive_callback); 

    ros::Rate loop_rate(10);

    while (ros::ok()){
	    ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}
