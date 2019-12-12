#include "../include/move_ur5_qt/collision_objects_mannager.hpp"
#include <algorithm>
namespace move_ur5_qt {
// function that return a co
moveit_msgs::CollisionObject create_primitive_collision_object(
    string frame_id, string obj_id, primitive_shape_set obj_shape,
    vector<double> obj_dimensions, vector<double> obj_pose) {
  moveit_msgs::CollisionObject obj;  // return obj
  obj.header.frame_id = frame_id;
  obj.id = obj_id;
  shape_msgs::SolidPrimitive primitive;  // obj size and type
  geometry_msgs::Pose pose;              // obj position and orientation

  primitive.type = obj_shape;
  primitive.dimensions = obj_dimensions;
  obj.primitives.push_back(primitive);  // set obj property

  pose.position.x = obj_pose[0];
  pose.position.y = obj_pose[1];
  pose.position.z = obj_pose[2];
  switch (obj_pose.size()) {
    case 7:
      pose.orientation.x = obj_pose[3];
      pose.orientation.y = obj_pose[4];
      pose.orientation.z = obj_pose[5];
      pose.orientation.w = obj_pose[6];
      break;
    case 6:
      geometry_msgs::Quaternion obj_q;
      tf::Quaternion q =
          tf::createQuaternionFromRPY(obj_pose[3], obj_pose[4], obj_pose[5]);
      tf::quaternionTFToMsg(q, obj_q);
      pose.orientation = obj_q;
      break;
  }
  obj.primitive_poses.push_back(pose);  // set pose
  obj.operation = obj.ADD;
  return obj;
}
// re-implentation
moveit_msgs::CollisionObject create_primitive_collision_object(
    collision_object collision_obj) {
  moveit_msgs::CollisionObject obj;  // return obj
  obj.header.frame_id = collision_obj.frame;
  obj.id = collision_obj.name;
  shape_msgs::SolidPrimitive primitive;  // obj size and type
  geometry_msgs::Pose pose;              // obj position and orientation

  primitive.type = collision_obj.shape;
  primitive.dimensions = collision_obj.dimensions;
  obj.primitives.push_back(primitive);  // set obj property

  vector<double> obj_pose = collision_obj.pose;
  pose.position.x = obj_pose[0];
  pose.position.y = obj_pose[1];
  pose.position.z = obj_pose[2];
  switch (obj_pose.size()) {
    case 7:
      pose.orientation.x = obj_pose[3];
      pose.orientation.y = obj_pose[4];
      pose.orientation.z = obj_pose[5];
      pose.orientation.w = obj_pose[6];
      break;
    case 6:
      geometry_msgs::Quaternion obj_q;
      tf::Quaternion q =
          tf::createQuaternionFromRPY(obj_pose[3], obj_pose[4], obj_pose[5]);
      tf::quaternionTFToMsg(q, obj_q);
      pose.orientation = obj_q;
      break;
  }
  obj.primitive_poses.push_back(pose);  // set pose
  obj.operation = obj.ADD;
  return obj;
}

// function that return a color
moveit_msgs::ObjectColor setColor(std::string name, float r, float g, float b,
                                  float a = 0.9f) {
  moveit_msgs::ObjectColor color;
  color.id = name;
  color.color.r = r;
  color.color.g = g;
  color.color.b = b;
  color.color.a = a;
  return color;
}

// re-implentation
moveit_msgs::ObjectColor setColor(std::string name, vector<float> rgba) {
  moveit_msgs::ObjectColor color;
  color.id = name;
  color.color.r = rgba[0];
  color.color.g = rgba[1];
  color.color.b = rgba[2];
  switch (rgba.size()) {
    case 3:
      color.color.a = 1.0f;
      break;
    case 4:
      color.color.a = rgba[3];
      break;
  }
  return color;
}

bool open_file(std::fstream &obj_file, string dir) {
  char *current_working_dir = getcwd(nullptr, 0);
  if (current_working_dir == nullptr) {
    ROS_ERROR("getcwd error");
    return false;
  }
  ROS_INFO("current working dir: %s, load file from: %s", current_working_dir,
           dir.c_str());
  free(current_working_dir);
  obj_file.open(dir, std::ios::in | std::ios::out);
  if (!obj_file) {
    ROS_ERROR("file not exist.");
    return false;
  }
  return true;
}

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

// class function
Collision_Objects_Mannager::Collision_Objects_Mannager(int argc, char **argv)
    : init_argc(argc), init_argv(argv) {}

Collision_Objects_Mannager::~Collision_Objects_Mannager() {}

void Collision_Objects_Mannager::init() {
  ros::init(init_argc, init_argv, "collision_objects_mannager_node");
  nh_ptr = std::make_shared<ros::NodeHandle>();
  ros::AsyncSpinner spin(1);
  spin.start();
  planning_scene_interface_ptr =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  ros::Duration(1.0).sleep();  // wait for init
  planning_scene_diff_publisher = nh_ptr->advertise<moveit_msgs::PlanningScene>(
      "/move_group/monitored_planning_scene", 1);
}

void Collision_Objects_Mannager::init(const std::string &master_url,
                                      const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "collision_objects_mannager_node");

  nh_ptr = std::make_shared<ros::NodeHandle>();
  ros::AsyncSpinner spin(1);
  spin.start();
  planning_scene_interface_ptr =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  ros::Duration(1.0).sleep();  // wait for init
  planning_scene_diff_publisher =
      nh_ptr->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

vector<string> Collision_Objects_Mannager::get_objects_names() {
  using std::pair;
  vector<string> names;
  for (pair<string, collision_object> _obj : collision_object_set) {
    names.push_back(_obj.first);
  }
  return names;
}

// rm obj
void Collision_Objects_Mannager::remove_object(string object_name) {
  if (!collision_object_set.erase(object_name))
    ROS_INFO("object %s not exist", object_name.c_str());
}

// rm all obj
void Collision_Objects_Mannager::remove_all_objects() {
  vector<string> objects_id_set = get_objects_names();
  planning_scene_interface_ptr->removeCollisionObjects(objects_id_set);
}

// update objects
void Collision_Objects_Mannager::update() {
  using std::pair;
  remove_all_objects();
  // get moveit object set, scence color set
  vector<moveit_msgs::CollisionObject> objects_set;
  planning_scene.object_colors.clear();
  planning_scene.world.collision_objects.clear();
  for (pair<string, collision_object> _obj : collision_object_set) {
    // get object
    moveit_msgs::CollisionObject primitive_collision_obj;
    primitive_collision_obj = create_primitive_collision_object(_obj.second);
    planning_scene.object_colors.push_back(
        setColor(_obj.second.name, _obj.second.color));
    planning_scene.world.collision_objects.push_back(primitive_collision_obj);
    objects_set.push_back(primitive_collision_obj);
  }
  // add objects
  planning_scene_interface_ptr->addCollisionObjects(objects_set);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ros::Duration(0.5).sleep();
  Q_EMIT Collision_Objects_Updated();
}

// add single obj
void Collision_Objects_Mannager::add_collision_object(collision_object obj) {
  using std::pair;
  ROS_INFO("Add %s to %s", obj.name.c_str(), obj.frame.c_str());
  // edit golbal set
  collision_object_set[obj.name] = obj;
  update();
  ROS_INFO("Done");
}

// add objects form vector
void Collision_Objects_Mannager::add_collision_object(
    vector<collision_object> obj_list) {
  for (collision_object obj : obj_list) add_collision_object(obj);
}

bool Collision_Objects_Mannager::load_collision_objects_form_file(string dir) {
  using std::cout;
  using std::fstream;
  using std::ios;
  map<string, primitive_shape_set> shape_map;
  shape_map["BOX"] = BOX;
  shape_map["SPHERE"] = SPHERE;
  shape_map["CYLINDER"] = CYLINDER;
  shape_map["CONE"] = CONE;

  fstream obj_file;
  if (!open_file(obj_file, dir)) return false;
  map<string, string> obj_arg;
  while (!obj_file.eof()) {
    string line;
    obj_file >> line;
    int p = line.find(':');
    if (p != string::npos) {
      string key(line, 0, p);
      string value(line, p + 1, line.size());
      obj_arg[key] = value;
    }
  }
  obj_file.close();
  vector<double> dim_v, pos_v;
  vector<float> col_v;
  if (obj_arg.size() != 6 || !string2vector(obj_arg["dimensions"], dim_v) ||
      !string2vector(obj_arg["color"], col_v) ||
      !string2vector(obj_arg["pose"], pos_v)) {
    ROS_ERROR("format error in collision object file.");
    return false;
  }
  collision_object obj;
  obj.frame = obj_arg["frame"];
  obj.name = obj_arg["name"];
  obj.shape = shape_map[obj_arg["shape"]];
  obj.dimensions = dim_v;
  obj.color = col_v;
  obj.pose = pos_v;
  collision_object_set[obj.name] = obj;
  ROS_INFO("object loaded: %s", dir.c_str());
  return true;
}

void Collision_Objects_Mannager::load_collision_objects_form_dir(
    string folder) {
  file_names fn = get_files(folder, "txt");
  if (fn.ext_files.empty()) {
    ROS_WARN("No objects in the folder: %s", folder.c_str());
    return;
  }
  vector<string>::iterator it = fn.ext_files.begin();
  for (; it != fn.ext_files.end(); ++it) {
    if (!load_collision_objects_form_file(*it))
      ROS_ERROR("Failed load object: %s from %s", it->c_str(), folder.c_str());
  }
}

}  // namespace move_ur5_qt
