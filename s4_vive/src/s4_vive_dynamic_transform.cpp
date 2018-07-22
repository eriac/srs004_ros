#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <s4_vive/transform_rpyConfig.h>

std::string source_frame="world";
std::string target_frame="base_link";
float xyz[3]={0,0,0};
float rpy[3]={0,0,0};

void broadcast(float *xyz, float *rpy){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(xyz[0], xyz[1] ,xyz[2]) );
  tf::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));
}

void callback(s4_vive::transform_rpyConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:\n %f %f %f \n%f %f %f", 
            config.pos_x, config.pos_x, config.pos_x, 
            config.roll,  config.pitch, config.yaw);

  xyz[0]=config.pos_x;
  xyz[1]=config.pos_y;
  xyz[2]=config.pos_z;
  rpy[0]=config.roll;
  rpy[1]=config.pitch;
  rpy[2]=config.yaw;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reconfigure");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.getParam("source_frame", source_frame);
  pn.getParam("target_frame", target_frame);

  dynamic_reconfigure::Server<s4_vive::transform_rpyConfig> server;
  dynamic_reconfigure::Server<s4_vive::transform_rpyConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

	ros::Rate loop_rate(100); 
	while (ros::ok()){
    broadcast(xyz, rpy);
    ROS_INFO("tf");

    ros::spinOnce();
		loop_rate.sleep();
	} 
  return 0;
}
