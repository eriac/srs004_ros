// OSS
#include <math.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/server/simple_action_server.h>
// SRS004
#include <s4_msgs/GameAppAction.h>

typedef actionlib::SimpleActionServer<s4_msgs::GameAppAction> Server;

class JoyFireControl{
public:
  JoyFireControl() : nh_(), pnh_("~"), server_(nh_, "joy_navigation", false) {
    cmd_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("output_point", 10);
    cmd_vel_sub_ = nh_.subscribe("input_twist", 10, &JoyFireControl::twistCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0/20.0), &JoyFireControl::timerCallback, this);

    server_.start();
    ROS_INFO("start");
  }

  void twistCallback(const geometry_msgs::Twist& twist_msg){
    last_twist_ = twist_msg;
  }

  void timerCallback(const ros::TimerEvent& event){
    if(last_twist_.linear.y != 0 || last_twist_.linear.z != 0){
      last_point_.point.x = 1.0;
      last_point_.point.y += last_twist_.linear.y /20.0;
      last_point_.point.z += last_twist_.linear.z /20.0;
      cmd_pos_pub_.publish(last_point_);
    }
  }

  bool updateState(void){
    s4_msgs::GameAppGoalConstPtr gg;
    if(server_.isNewGoalAvailable()){
      gg=server_.acceptNewGoal();
      ROS_INFO("Update Goal\n");
    }
    if(server_.isActive()){
      if(server_.isPreemptRequested()){
        //server_.setAborted();
        server_.setPreempted();
        ROS_INFO("Preempt Goal\n");
        stopFireControl();
        return false;
      }
      return true;
    }
    return false;
  }

  void stopFireControl(void){
    geometry_msgs::PointStamped zero_point_msg;
    cmd_pos_pub_.publish(zero_point_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  Server server_;
  ros::Publisher cmd_pos_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Timer timer_;

  geometry_msgs::PointStamped last_point_;
  geometry_msgs::Twist last_twist_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "joy_navigation");
  JoyFireControl joy_navigation;
  ros::spin();
}

// int main(int argc, char** argv){
//   ros::init(argc, argv, "s4_omni_move_server");
//   ros::NodeHandle nh;
//   ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

//   Server server(nh, "move_action", false);
//   server.start();

//   geometry_msgs::Twist last_twist;
//   ros::Rate loop_rate(10);
//   float dt=1.0/10;
//   s4_msgs::GameAppGoalConstPtr goal;
//   while (ros::ok()){
//     if(server.isNewGoalAvailable()){
//       goal=server.acceptNewGoal();
//     }

//     if(server.isActive()){
//       ros::Time end=goal->header.stamp+ros::Duration(goal->duration);
//       if(end<ros::Time::now()){
//         geometry_msgs::Twist zero_twist;
//         cmd_pub.publish(zero_twist);
//         ROS_INFO("twist (%f, %f)", (float)0, (float)0);
//         last_twist=zero_twist;
//         server.setSucceeded();
//       }
//       else{//action
//         if(goal->mode == s4_msgs::GameAppGoal::NAV_VEL){
//           double linear_dist=sqrt(pow(goal->twist.linear.x-last_twist.linear.x,2)+pow(goal->twist.linear.y-last_twist.linear.y,2));
//           //double linear_dist=(goal->twist.linear.x-last_twist.linear.x);
//           if(linear_dist<=limit_linear_acc*dt){
//             cmd_pub.publish(goal->twist);
//             ROS_INFO("twist (%f, %f)", goal->twist.linear.x, goal->twist.linear.y);
//             last_twist=goal->twist;
//           }
//           else{
//             geometry_msgs::Twist current_twist;
//             float rate=(limit_linear_acc*dt)/linear_dist;
//             ROS_INFO("modify %f/%f", linear_dist, limit_linear_acc*dt);
//             current_twist.linear.x=last_twist.linear.x+(goal->twist.linear.x-last_twist.linear.x)*rate;
//             current_twist.linear.y=last_twist.linear.y+(goal->twist.linear.y-last_twist.linear.y)*rate;
//             current_twist.angular.z=goal->twist.angular.z;
//             cmd_pub.publish(current_twist);
//             ROS_INFO("twist (%f, %f)", current_twist.linear.x, current_twist.linear.y);
//             last_twist=current_twist;
//           }
//         }
//       }
//     }  
//     ros::spinOnce();
//     loop_rate.sleep();
//   } 
//   return 0;
// }