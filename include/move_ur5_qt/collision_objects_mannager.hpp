#ifndef COLLISION_OBJECTS_MANNAGER_HPP
#define COLLISION_OBJECTS_MANNAGER_HPP

#ifndef Q_MOC_RUN
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/tf.h>
#endif
#include <string>
#include <vector>
#include <QObject>
#include <fstream>
#include <math.h>
#include "getfile.hpp"
#include "globalobj.hpp"
#include "globaldata.hpp"
#include "comannagerdialog.hpp"
using std::map;
using std::string;
using std::vector;

// function declear
moveit_msgs::CollisionObject create_primitive_collision_object(
    string frame_id, string obj_id, primitive_shape_set obj_shape,
    vector<double> obj_dimensions, vector<double> obj_pose);
moveit_msgs::CollisionObject create_primitive_collision_object(
    collision_object collision_obj);
moveit_msgs::ObjectColor setColor(std::string name, float r, float g, float b,
                                  float a);
moveit_msgs::ObjectColor setColor(std::string name, vector<float> rgba);
bool open_file(std::fstream &obj_file, string dir);
// convert [12,23,323]
template <typename T>
bool string2vector(string str, vector<T> &vec) {
  if (str.empty()) return false;
  string sub_str(str, 1, str.size() - 1);  // 12,23,323

  vector<string> n_list;
  string::iterator it = sub_str.begin();
  string n = "";
  for (; it != sub_str.end(); it++) {
    if (*it == ',' || it == sub_str.end() - 1) {
      n_list.push_back(n);
      n = "";
      continue;
    }
    n += *it;
  }
  for (string s : n_list) {
    T num;
    std::stringstream ss;
    ss << s;
    ss >> num;  // convert to number
    vec.push_back(num);
  }
  return true;
}

template <typename T>
vector<T> string2vector(string str) {
  vector<T> vec;
  if (!str.empty()) {
    string sub_str(str, 1, str.size() - 1);  // 12,23,323
    vector<string> n_list;
    string::iterator it = sub_str.begin();
    string n = "";
    for (; it != sub_str.end(); it++) {
      if (*it == ',' || it == sub_str.end() - 1) {
        n_list.push_back(n);
        n = "";
        continue;
      }
      n += *it;
    }
    for (string s : n_list) {
      T num;
      std::stringstream ss;
      ss << s;
      ss >> num;  // convert to number
      vec.push_back(num);
    }
  }
  return vec;
}

// class
class Collision_Objects_Mannager : public QObject {
  Q_OBJECT
 public:
  Collision_Objects_Mannager();
  virtual ~Collision_Objects_Mannager();
  void init();
  void update();
  void add_collision_object(collision_object obj, bool log = true);
  void add_collision_object(vector<collision_object> obj_list);
  bool load_collision_objects_form_file(string dir);
  void load_collision_objects_form_dir(string folder);
  bool isvalid(collision_object obj);
  vector<string> get_objects_names();
  collision_object *get_named_object(string name);
  void remove_object(string object_name);
  void remove_all_objects();
  void modify_object_pose(string name, vector<double> pose);
  string get_object_info(collision_object);
Q_SIGNALS:
  void Collision_Objects_Updated();

 private:
  std::shared_ptr<ros::NodeHandle> nh_ptr;
  moveit::planning_interface::PlanningSceneInterfacePtr
      planning_scene_interface_ptr;
  ros::Publisher planning_scene_diff_publisher;
  // planning scene
  //  vector<std::string> objects_id_set;
  moveit_msgs::PlanningScene planning_scene;
  //  map<string, moveit_msgs::CollisionObject> collision_object_set;
  map<string, collision_object> collision_object_set;
};

#endif  // COLLISION_OBJECTS_MANNAGER_HPP
