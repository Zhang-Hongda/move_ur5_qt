#ifndef COLLISION_OBJECTS_MANNAGER_H
#define COLLISION_OBJECTS_MANNAGER_H

#include <fstream>
#include "getfile.h"
#include <math.h>
#ifndef Q_MOC_RUN
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/tf.h>
#endif
#include <string>
#include <vector>

namespace move_ur5_qt {

using std::map;
using std::string;
using std::vector;

// basic shape
enum primitive_shape_set {
  BOX = 1u,
  SPHERE = 2u,
  CYLINDER = 3u,
  CONE = 4u,
};

// all properties of a co
struct collision_object {
  string frame;
  string name;
  primitive_shape_set shape;
  vector<float> color;
  vector<double> dimensions;
  vector<double> pose;
  collision_object() {}
  collision_object(string _frame, string _name, primitive_shape_set _shape,
                   vector<float> _color, vector<double> _dimensions,
                   vector<double> _pose) {
    frame = _frame;
    name = _name;
    shape = _shape;
    color = _color;
    dimensions = _dimensions;
    pose = _pose;
  }
};

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

// class
class Collision_Objects_Mannager {
 public:
  Collision_Objects_Mannager(int argc, char **argv);
  ~Collision_Objects_Mannager();
  void init();
  void init(const std::string &master_url, const std::string &host_url);
  void add_collision_object(collision_object obj);
  void add_collision_object(vector<collision_object> obj_list);
  bool load_collision_objects_form_file(collision_object &obj, string dir);
  void load_collision_objects_form_dir(vector<collision_object> &obj_list,
                                       string folder);
  void remove_object(string object_name);
  void remove_all_objects();

 private:
  int init_argc;
  char **init_argv;
  std::shared_ptr<ros::NodeHandle> nh_ptr;
  moveit::planning_interface::PlanningSceneInterfacePtr
      planning_scene_interface_ptr;
  ros::Publisher planning_scene_diff_publisher;
  // planning scene
  vector<std::string> objects_id_set;
  moveit_msgs::PlanningScene planning_scene;
  map<string, moveit_msgs::CollisionObject> collision_object_set;
};
}  // namespace move_ur5_qt
#endif  // COLLISION_OBJECTS_MANNAGER_H
