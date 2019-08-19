#include "s4_hardware/usb_adapter.h"
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class GY955 : public USBSerial{
 public:
  GY955();
  bool process(std::deque<unsigned char>& buffer);
  void timerCallback(const ros::TimerEvent& event);
  bool exact(std::deque<unsigned char>& buffer);
  sensor_msgs::Imu convert_data(std::vector<unsigned char> data);
  bool checkSum(std::vector<unsigned char> data);
  bool checkSize(sensor_msgs::Imu imu_msg);

  ros::Publisher imu_pub_;
  ros::Timer timer_dummy_;

  std::string imu_frame_name_;
  std::vector<double> orientation_covariance_;
  bool debug_;
};

GY955::GY955(): USBSerial{}{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("data", 10);
  timer_dummy_ = nh_.createTimer(ros::Duration(1.0), &GY955::timerCallback, this);

  imu_frame_name_ = "imu_frame";
  pnh_.getParam("imu_frame_name", imu_frame_name_);
  orientation_covariance_.resize(3);
  pnh_.getParam("orientation_covariance", orientation_covariance_);
}

bool GY955::process(std::deque<unsigned char>& buffer){
  exact(buffer);
}

void GY955::timerCallback(const ros::TimerEvent& event){
  std::vector<unsigned char> activate_data({170, 16, 10});
  output(activate_data);
}

bool GY955::exact(std::deque<unsigned char>& buffer){
  //extract
  for (int i = 0; i < (int)buffer.size() - 14; i++){
    std::vector<unsigned char> data;
    if (buffer[i] == 90 && buffer[i + 1] == 90 && buffer[i + 2] == 16 && buffer[i + 3] == 9){
      for (int j = i; j < i + 14; j++){
        data.push_back(buffer[j]);
      }

      for (int j = 0; j < i + 14; j++){
        buffer.pop_front();
      }

      if (checkSum(data)){
        sensor_msgs::Imu imu_data = convert_data(data);
        if (checkSize(imu_data))
          imu_pub_.publish(imu_data);
        else
          ROS_ERROR_THROTTLE(5.0, "NG size");
      }
      else
        ROS_ERROR_THROTTLE(5.0, "NG checksum");
        //print_data(data);
      break;
    }
  }
}

sensor_msgs::Imu GY955::convert_data(std::vector<unsigned char> data){
  int mode = data[2];
  int size = data[3];
  int16_t tmp_w = (data[4] * 256 + data[5]);
  float quat_w = tmp_w / 10000.0;
  int16_t tmp_x = (data[6] * 256 + data[7]);
  float quat_x = tmp_x / 10000.0;
  int16_t tmp_y = (data[8] * 256 + data[9]);
  float quat_y = tmp_y / 10000.0;
  int16_t tmp_z = (data[10] * 256 + data[11]);
  float quat_z = tmp_z / 10000.0;
  int d0 = (data[12] >> 6) & 0x03;
  int d1 = (data[12] >> 4) & 0x03;
  int d2 = (data[12] >> 2) & 0x03;
  int d3 = (data[12] >> 0) & 0x03;
  float quat_size = quat_w * quat_w + quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = imu_frame_name_;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.orientation.x = quat_x;
  imu_msg.orientation.y = quat_y;
  imu_msg.orientation.z = quat_z;
  imu_msg.orientation.w = quat_w;
  if(orientation_covariance_.size()==3){
    imu_msg.orientation_covariance[0] = orientation_covariance_[0];
    imu_msg.orientation_covariance[4] = orientation_covariance_[1];
    imu_msg.orientation_covariance[8] = orientation_covariance_[2];
  }
  else ROS_ERROR_THROTTLE(5.0, "covariance size is not 3 but %i", (int)orientation_covariance_.size());
  return imu_msg;
}

bool GY955::checkSum(std::vector<unsigned char> data){
  if ((int)data.size() >= 2){
    int sum = 0;
    for (int i = 0; i < (int)data.size() - 1; i++){
      sum += data[i];
    }
    if (sum % 256 == data.back() % 256){
      // ROS_ERROR("OK");
      // for (auto d: data){
      //   printf("%i,", d);
      // }
      // printf("\n");
      return true;
    }
    else{
      // ROS_ERROR("sum:%i, last:%i", sum % 256, data.back() % 256);
      // for (auto d: data){
      //   printf("%i,", d);
      // }
      // printf("\n");
      return false;
    }
  }
  return false;
}

bool GY955::checkSize(sensor_msgs::Imu imu_msg){
  float x2 = imu_msg.orientation.x * imu_msg.orientation.x;
  float y2 = imu_msg.orientation.y * imu_msg.orientation.y;
  float z2 = imu_msg.orientation.z * imu_msg.orientation.z;
  float w2 = imu_msg.orientation.w * imu_msg.orientation.w;
  float size2 = x2 + y2 + z2 + w2;
  if (0.9 < size2 && size2 < 1.1)
    return true;
  return false;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "usb_test");
  GY955 usb_serial_test{};
  ros::spin();
}
