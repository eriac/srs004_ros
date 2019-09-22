// OSS
#include <cmath>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/ObjectArray.h>
// SRS004
#include <s4_detection/extractorConfig.h>
#include <s4_msgs/TrackedRayArray.h>
#include <s4_msgs/TrackedRectArray.h>

class HSVExtractor{

public:
  HSVExtractor()
    : nh_(), pnh_("~"), it_(nh_) {

    debug_ = false;
    debug_type_ = 0;
    reduction_ = 1;
    h_min_ = 50;
    h_max_ = 150;
    s_min_ = 50;
    s_max_ = 150;
    v_min_ = 50;
    v_max_ = 150;
    close_size_ = 3;
    open_size_ = 3;
    pnh_.getParam("debug", debug_);
    pnh_.getParam("debug_type", debug_type_);
    pnh_.getParam("reduction", reduction_);
    pnh_.getParam("h_min", h_min_);
    pnh_.getParam("h_max", h_max_);
    pnh_.getParam("s_min", s_min_);
    pnh_.getParam("s_max", s_max_);
    pnh_.getParam("v_min", v_min_);
    pnh_.getParam("v_max", v_max_);
    pnh_.getParam("close_size", close_size_);
    pnh_.getParam("open_size", open_size_);

    size_weight_ = 1.0;
    pnh_.getParam("size_weight", size_weight_);

    frame_id_ = "default";
    camera_name_ = "default";
    category_ = "default";
    pnh_.getParam("frame_id", frame_id_);
    pnh_.getParam("camera_name", camera_name_);
    pnh_.getParam("category", category_);

    if(debug_)debug_pub_ = it_.advertise("image_debug", 1);
    rays_pub_ = nh_.advertise<s4_msgs::TrackedRectArray>("tracked_rays", 10);
    camera_sub_ = it_.subscribeCamera("image_raw", 1, &HSVExtractor::imageCallback, this);
    
    if(debug_){
      reconfigure_f_ = boost::bind(&HSVExtractor::reconfigureCallback, this, _1, _2);
      reconfigure_server_.setCallback(reconfigure_f_);
    }

    last_id_ = 0;
    last_rects_.rects.clear();
  }

  void reconfigureCallback(s4_detection::extractorConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request level: %i", level);
    debug_type_ = config.type;
    reduction_ = config.reduction;
    h_min_ = config.h_min;
    h_max_ = config.h_max;
    s_min_ = config.s_min;
    s_max_ = config.s_max;
    v_min_ = config.v_min;
    v_max_ = config.v_max;
    close_size_ = config.close_size;
    open_size_ = config.open_size;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg){
    cv::Mat src_image, debug_image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      src_image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("Failed to convert image");
      return;
    }

    cv::Mat mid0_image = reduceSize(src_image);
    cv::Mat open_mask = generateMask(mid0_image);
    cv::Mat masked_image;
    cv::bitwise_and(mid0_image, mid0_image, masked_image, open_mask);

    if(debug_ && debug_type_ == 0){
      debug_image = masked_image;
    }

    std::vector<cv::Rect> reduction_rects, original_rects;
    extractRects(open_mask, reduction_rects);    
    resumeRects(reduction_rects, original_rects);

    if(debug_ && debug_type_ == 1){
      debug_image = src_image;
      for(cv::Rect rect : original_rects){
        cv::rectangle(debug_image, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), cv::Scalar(255, 0, 0), 3);
      }
    }
    updateTracking(original_rects);

    if(debug_ && debug_type_ == 2){
      debug_image = src_image;
      for(s4_msgs::TrackedRect rect : last_rects_.rects){      
        cv::putText(debug_image, std::to_string(rect.info.id).c_str(), cv::Point(rect.rect.x, rect.rect.y - 10), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255,0,0), 2, CV_AA);
        cv::rectangle(debug_image, cv::Point(rect.rect.x, rect.rect.y), cv::Point(rect.rect.x + rect.rect.width, rect.rect.y + rect.rect.height), cv::Scalar(255, 0, 0), 3);
      }
    }

    cam_model_.fromCameraInfo(*info_msg);
    for(s4_msgs::TrackedRect& rect : last_rects_.rects){
      cv::Point2d uv;
      uv.x = rect.rect.x + rect.rect.width / 2.0;
      uv.y = rect.rect.y + rect.rect.height / 2.0;
      cv::Point3d xyz = cam_model_.projectPixelTo3dRay(uv);
      rect.ray.x = xyz.x;
      rect.ray.y = xyz.y;
      rect.ray.z = xyz.z;
    }

    last_rects_.header.frame_id = frame_id_;
    last_rects_.header.stamp = ros::Time::now();
    last_rects_.camera_name = camera_name_;   
    rays_pub_.publish(last_rects_);

    if(debug_){
      cv_bridge::CvImage output_bridge;
      output_bridge.header = input_bridge->header;
      output_bridge.encoding = input_bridge->encoding;
      output_bridge.image = debug_image;
      debug_pub_.publish(output_bridge.toImageMsg());
    }
  }

  cv::Mat reduceSize(cv::Mat input){
    cv::Mat output;
    if(reduction_ == 1)output = input;
    else{
      int red_w = input.size().width / reduction_;
      int red_h = input.size().height / reduction_;
      cv::resize(input, output, cv::Size(red_w, red_h), 0, 0, cv::INTER_NEAREST);
    }
    return output;
  }

  cv::Mat generateMask(cv::Mat input){
    cv::Mat mid1_image;
    cv::cvtColor(input, mid1_image, CV_BGR2HSV);

    cv::Mat hsv_mask;	  
    if(h_min_ <= h_max_){
      cv::inRange(mid1_image, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), hsv_mask);
    }
    else{
      cv::Mat hsv_mask1, hsv_mask2;
      cv::inRange(mid1_image, cv::Scalar(0, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), hsv_mask1);
      cv::inRange(mid1_image, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(180, s_max_, v_max_), hsv_mask2);
      hsv_mask = hsv_mask1 + hsv_mask2;
    }

    cv::Mat close_mask;
    if(close_size_ == 0) close_mask = hsv_mask;
    else{
      int kernel_size = 1 + 2 * close_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(kernel_size, kernel_size));
      cv::Mat mid_mask;
      cv::dilate(hsv_mask, mid_mask, kernel);
      cv::erode(mid_mask, close_mask, kernel);
    }

    cv::Mat open_mask;
    if(open_size_ == 0) open_mask = close_mask;
    else{
      int kernel_size = 1 + 2 * open_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(kernel_size, kernel_size));
      cv::Mat mid_mask;
      cv::erode(close_mask, mid_mask, kernel);
      cv::dilate(mid_mask, open_mask, kernel);
    }
    return open_mask;
  }

  void extractRects(cv::Mat mask, std::vector<cv::Rect>& rects){
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    rects.clear();
    for(int i=0; i<(int)contours.size();i++){
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contours[i]), hull);
      rects.push_back(cv::boundingRect(cv::Mat(hull)));
    }
  }
  void resumeRects(std::vector<cv::Rect> reduction_rects, std::vector<cv::Rect>& original_rects){
    original_rects.clear();
    for(cv::Rect rect : reduction_rects){
      original_rects.push_back(cv::Rect(rect.x * reduction_, rect.y * reduction_, rect.width * reduction_, rect.height * reduction_));
    }
  }

  void updateTracking(std::vector<cv::Rect> original_rects){
    //tracking
    if(last_rects_.rects.empty()){
      for(cv::Rect rect_cv : original_rects){
        if(rect_cv.width > 10 && rect_cv.height > 10){
          s4_msgs::TrackedRect rect_msg;
          rect_msg.info.category = category_;
          rect_msg.info.id = last_id_;
          last_id_++;
          rect_msg.info.age = 0;
          rect_msg.info.category = category_;
          rect_msg.rect.x = rect_cv.x;
          rect_msg.rect.y = rect_cv.y;
          rect_msg.rect.width = rect_cv.width;
          rect_msg.rect.height = rect_cv.height;
          last_rects_.rects.push_back(rect_msg);
        }
      }
    }
    else if(original_rects.empty()){
      last_rects_.rects.clear();
    }
    else{
      int old_size = last_rects_.rects.size();
      int new_size = original_rects.size();
      std::vector<std::vector<float> > relation_matrix;
      relation_matrix.resize(old_size);
      for(int i = 0; i< old_size; i++){
        relation_matrix[i].resize(new_size);
      }

      for(int i = 0; i< old_size; i++){
        for(int j = 0; j< new_size; j++){
          relation_matrix[i][j] = calcDistance(original_rects[j], last_rects_.rects[i].rect);
        }
      }

      // for(int i = 0; i< old_size; i++){
      //   for(int j = 0; j< new_size; j++){
      //     printf("%f, ", relation_matrix[i][j]);
      //   }
      //   printf("\n");
      // }

      std::vector<int> relation_new;
      relation_new.resize(new_size);
      for(int j = 0; j< new_size; j++)relation_new[j] = -1;

      float distance_threshold = 400;
      for(int k = 0; k< old_size; k++){
        int min_i, min_j;
        getMinimumIndex(relation_matrix, min_i, min_j);
        if(relation_matrix[min_i][min_j] < distance_threshold){
          relation_new[min_j] = min_i;
        }

        for(int i = 0; i< old_size; i++)relation_matrix[i][min_j] = 10000;
        for(int j = 0; j< new_size; j++)relation_matrix[min_i][j] = 10000;
      }

      // for(int j = 0; j< new_size; j++){
      //   printf("%i, ", relation_new[j]);
      // }
      // printf("\n\n");

      s4_msgs::TrackedRectArray next_rects;
      for(int j = 0; j < new_size; j++){
        cv::Rect rect_cv = original_rects[j];
        if(relation_new[j] < 0){
          if(rect_cv.width > 10 && rect_cv.height > 10){
            s4_msgs::TrackedRect rect_msg;
            rect_msg.info.category = category_;
            rect_msg.info.id = last_id_;
            last_id_++;
            rect_msg.info.age = 0;
            rect_msg.info.category = category_;
            rect_msg.rect.x = rect_cv.x;
            rect_msg.rect.y = rect_cv.y;
            rect_msg.rect.width = rect_cv.width;
            rect_msg.rect.height = rect_cv.height;
            next_rects.rects.push_back(rect_msg);
          }
        }
        else{
          s4_msgs::TrackedRect rect_msg = last_rects_.rects[relation_new[j]];
          rect_msg.info.age++;
          rect_msg.rect.x = rect_cv.x;
          rect_msg.rect.y = rect_cv.y;
          rect_msg.rect.width = rect_cv.width;
          rect_msg.rect.height = rect_cv.height;
          next_rects.rects.push_back(rect_msg);
        }
      }
      last_rects_ = next_rects;
    }
  }

  float calcDistance(cv::Rect rect1, jsk_recognition_msgs::Rect rect2){
    float dx = (rect1.x + rect1.width / 2) - (rect2.x + rect2.width / 2);
    float dy = (rect1.y + rect1.height / 2) - (rect2.y + rect2.height / 2);
    float dw = rect1.width - rect2.width;
    float dh = rect1.height - rect2.height;
    return sqrt(dx * dx + dy * dy) + size_weight_ * sqrt(dw * dw + dh * dh);
  }

  void getMinimumIndex(std::vector<std::vector<float> > relation_matrix, int& i, int& j){
    float distance_threshold = 400;
    int old_size = relation_matrix.size();
    int new_size = relation_matrix[0].size();
    
    std::vector<float> old_min, old_index;
    old_min.resize(old_size);
    old_index.resize(old_size);

    for(int i = 0; i< old_size; i++){
      auto iter = std::min_element(relation_matrix[i].begin(), relation_matrix[i].end());
      size_t index = std::distance(relation_matrix[i].begin(), iter);
      old_min[i] =  relation_matrix[i][index];
      old_index[i] = index;
    }
    
    auto iter = std::min_element(old_min.begin(), old_min.end());
    size_t index = std::distance(old_min.begin(), iter);
    i = index;
    j = old_index[i];
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher rays_pub_;
  image_transport::Publisher debug_pub_;

  bool debug_;
  int reduction_;
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;
  int close_size_;
  int open_size_;
  int debug_type_;
  std::string frame_id_;
  std::string camera_name_;
  std::string category_;
  double size_weight_;

  dynamic_reconfigure::Server<s4_detection::extractorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<s4_detection::extractorConfig>::CallbackType reconfigure_f_;
  image_geometry::PinholeCameraModel cam_model_;

  s4_msgs::TrackedRectArray last_rects_;
  int last_id_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "hsv_extractor");
  HSVExtractor hsv_extractor;
  ros::spin();
  return 0;
}