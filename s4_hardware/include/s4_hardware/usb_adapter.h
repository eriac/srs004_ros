// OSS
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <deque>
#include <string>
// ROS
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

class USBSerial{
 public:
  USBSerial();

 protected:
  virtual bool process(std::deque<unsigned char>& buffer);
  void output(const std::vector<unsigned char>& output_data);
  void timer1Callback(const ros::TimerEvent& event);
  void timer2Callback(const ros::TimerEvent& event);
  std::deque<unsigned char> buffer_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

 private:
  void openSerial(void);
  void closeSerial(void);
  std::vector<unsigned char> readSerial(void);
  void writeSerial(const std::vector<unsigned char>& data);
  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
  ros::Timer timer1_;
  ros::Timer timer2_;
  diagnostic_updater::Updater updater_;

  int fd_;
  std::string device_name_;
  int baud_rate_;
  bool connected_;
  int read_bytes_;
  int close_reason_;
};

USBSerial::USBSerial() : nh_(), pnh_("~"){
  connected_ = false;
  read_bytes_ = 0;
  close_reason_ = 0;

  device_name_ = "/dev/ttyUSB0";
  pnh_.getParam("device_name", device_name_);

  baud_rate_ = 9600;
  pnh_.getParam("baud_rate", baud_rate_);

  openSerial();

  if(connected_) ROS_INFO("Connect: %s", device_name_.c_str());
  else ROS_ERROR("Fail open: %s", device_name_.c_str());

  timer1_ = nh_.createTimer(ros::Duration(0.01), &USBSerial::timer1Callback, this);
  timer2_ = nh_.createTimer(ros::Duration(1.00), &USBSerial::timer2Callback, this);

  updater_.setHardwareID("SerialPort");
	updater_.add("Connect",  boost::bind(&USBSerial::diag_callback, this, _1));

}

bool USBSerial::process(std::deque<unsigned char>& buffer){
  //print & clear
  ROS_WARN("INPUT");
  for(auto c: buffer){
    if(isprint(c))printf("%c", c);
    else printf("*");
  }
  printf("\n");
  buffer.clear();
  return true;
}

void USBSerial::output(const std::vector<unsigned char>& send_data){
  writeSerial(send_data);
}

void USBSerial::timer1Callback(const ros::TimerEvent& event){
  if(connected_){
    for(auto c : readSerial()){
      buffer_.push_back(c);
    }
    if(!buffer_.empty()){
      bool ret = process(buffer_);
    }
  }
  else{
    openSerial();
    if(connected_) ROS_INFO("ReConnect: %s", device_name_.c_str());
  }

}

void USBSerial::timer2Callback(const ros::TimerEvent& event){
  updater_.update();
  read_bytes_ = 0;
}

void USBSerial::openSerial(void){
  fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(fd_, F_SETFL, 0);
  // load configuration
  struct termios conf_tio;
  tcgetattr(fd_, &conf_tio);
  // set baudrate
  speed_t BAUDRATE;
  if (baud_rate_ == 9600) {
    BAUDRATE = B9600;
  } else if (baud_rate_ == 1000000) {
    BAUDRATE = B1000000;
  } else {
    connected_ = false;
    close_reason_ = 1;
    return;
  }
  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);
  // non canonical, non echo back
  conf_tio.c_lflag &= CS8;
  // conf_tio.c_lflag &= ~(ECHO | ICANON);
  // non blocking
  conf_tio.c_cc[VMIN] = 0;
  conf_tio.c_cc[VTIME] = 0;
  // store configuration
  tcsetattr(fd_, TCSANOW, &conf_tio);
  if (fd_ >= 0) {
    connected_ = true;
    close_reason_ = 0;
  } else {
    connected_ = false;
    close_reason_ = 1;
  }
}

void USBSerial::closeSerial(void){
  close(fd_);
  connected_ = false;
  close_reason_ = 0;
}


std::vector<unsigned char> USBSerial::readSerial(void) {
  if (connected_) {
    char buf[256] = {0};
    int recv_size = read(fd_, buf, sizeof(buf));
    if (recv_size > 0) {
      read_bytes_ += recv_size;
      std::vector<unsigned char> recv_data;
      recv_data.resize(recv_size);
      for (int i = 0; i < recv_size; i++) recv_data[i] = buf[i];
      return recv_data;
    }
  }
  std::vector<unsigned char> null_data;
  return null_data;
}

void USBSerial::writeSerial(const std::vector<unsigned char>& send_data){
  if (connected_ && (int)send_data.size() > 0) {
    int rec = write(fd_, send_data.data(), (int)send_data.size());
    if (rec <= 0) {
      ROS_ERROR("Disconnect: %s", device_name_.c_str());
      connected_ = false;
      close_reason_ = 2;
      closeSerial();
    }
  }
}

void USBSerial::diag_callback(diagnostic_updater::DiagnosticStatusWrapper &stat){
  bool serial_c = connected_;
  bool serial_s = read_bytes_ > 0;
  if     (serial_c &&  serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,    "Active.");
  else if(serial_c && !serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,  "No Recieve.");
  else		                  stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
}
