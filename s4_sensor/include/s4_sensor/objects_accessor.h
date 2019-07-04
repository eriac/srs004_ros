#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <s4_msgs/GameAppAction.h>
#include <s4_msgs/TrackedInfo.h>
#include <s4_msgs/TrackedObjectArray.h>

#include <map>

class ObjectsAccessor{
public:
  ObjectsAccessor();
  ~ObjectsAccessor() = default;
  void UpdateObjects(const s4_msgs::TrackedObjectArray& objects) ;
  bool GetFocus(s4_msgs::TrackedInfo& info)const;
  bool FocusExist(void);

  bool GetCategories(std::vector<std::string>& categories)const;
  bool SetCategory(const std::string& category);
  bool SetShift(const int shift);
  bool SetNearest(geometry_msgs::Point refferece_point);
  
private:
  s4_msgs::TrackedObjectArray filterByCategory(const s4_msgs::TrackedObjectArray& objects, const std::string& focus_category)const;

  s4_msgs::TrackedObjectArray objects_;
  s4_msgs::TrackedInfo focus_info_;
  std::string focus_category_;
};

ObjectsAccessor::ObjectsAccessor(){
}

void ObjectsAccessor::UpdateObjects(const s4_msgs::TrackedObjectArray& objects){
  objects_ = objects;
}

bool ObjectsAccessor::GetFocus(s4_msgs::TrackedInfo& info)const{
  if(focus_info_.category.size() != 0){
    info = focus_info_;
    return true;
  }
  else{
    return false;
  }
}

bool ObjectsAccessor::FocusExist(void){
  for(int i =0; i < objects_.objects.size(); i++){
    if(objects_.objects[i].info.category == focus_info_.category){
      if(objects_.objects[i].info.id == focus_info_.id){
        return true;
      }
    }
  }
  return false;
}

bool ObjectsAccessor::GetCategories(std::vector<std::string>& categories)const{
  if(objects_.objects.size() != 0){
    std::map<std::string,int> category_list;
    for(int i =0; i < objects_.objects.size(); i++){
      category_list[objects_.objects[i].info.category]++;
    }

    categories.resize(0);

    auto begin = category_list.begin(), end = category_list.end();
    for (auto iter = begin; iter != end; iter++) {
      categories.push_back(iter->first);
    }
    return true;
  }
  else{
    return false;
  }
}

bool ObjectsAccessor::SetCategory(const std::string& category){
  focus_category_ = category;
}

bool ObjectsAccessor::SetNearest(geometry_msgs::Point refferece_point){
  s4_msgs::TrackedObjectArray focus_objects = filterByCategory(objects_, focus_category_);
  std::vector<double> score;
  for(int i =0; i < objects_.objects.size(); i++){
    double dx = focus_objects.objects[i].center.x - refferece_point.x;
    double dy = focus_objects.objects[i].center.y - refferece_point.y;
    double dz = focus_objects.objects[i].center.z - refferece_point.z;
    double dist = dx * dx + dy * dy + dz * dz;
    score.push_back(dist);
  }
  if(!focus_objects.objects.empty()){
    std::vector<double>::iterator minIt = std::min_element(score.begin(), score.end());
    size_t minIndex = std::distance(score.begin(), minIt);
    focus_info_ = focus_objects.objects[minIndex].info;
    return true;
  }
  return false;
}

bool ObjectsAccessor::SetShift(const int shift){
  s4_msgs::TrackedObjectArray focus_objects = filterByCategory(objects_, focus_info_.category);
  int index = -1;
  for(int i =0; i < objects_.objects.size(); i++){
    if(focus_objects.objects[i].info.id == focus_info_.id){
      index = i;
      break;
    }
  }
  if(index >= 0){
    int next_index = (index + shift) % focus_objects.objects.size();
    focus_info_ = focus_objects.objects[next_index].info; 
    return true;
  }
  else{
    return false;
  }
}

//
// private
//
s4_msgs::TrackedObjectArray ObjectsAccessor::filterByCategory(const s4_msgs::TrackedObjectArray& objects, const std::string& focus_category)const{
  s4_msgs::TrackedObjectArray focus_objects;
  for(int i =0; i < objects.objects.size(); i++){
    if(objects.objects[i].info.category == focus_category){
      focus_objects.objects.push_back(objects.objects[i]);
    }
  }
  return focus_objects;
}