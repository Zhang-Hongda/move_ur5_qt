#include "../include/move_ur5_qt/objectdrawer.hpp"
#include "../include/move_ur5_qt/globalobj.hpp"
#include <math.h>

ObjectDrawer::ObjectDrawer() {}
ObjectDrawer::~ObjectDrawer() {
  delete obj;
  obj = nullptr;
}

void ObjectDrawer::draw_object(collision_object *obj_ptr) {
  stopsign = false;
  obj = obj_ptr;
  start();
}

void ObjectDrawer::draw_box(geometry_msgs::Pose startPose) {
  obj->shape = BOX;
  double start_x = startPose.position.x;
  double start_y = startPose.position.y;
  double start_z = startPose.position.z;
  obj->pose = {start_x, start_y, start_z, 0, 0, 0};
  while (!stopsign) {
    geometry_msgs::Pose currentPose = listener.getMarkerposition();
    double current_x = currentPose.position.x;
    double current_y = currentPose.position.y;
    double current_z = currentPose.position.z;
    obj->pose = {start_x, start_y, (current_z - start_z) / 2 + start_z, 0, 0,
                 0};
    obj->dimensions = {2 * (current_x - start_x), 2 * (current_y - start_y),
                       current_z - start_z};
    Q_EMIT
    dimensions_updated(obj->dimensions);
    collision_objects_mannager.add_collision_object(*obj, false);
  }
}

void ObjectDrawer::draw_sphere(geometry_msgs::Pose startPose) {
  obj->shape = SPHERE;
  double start_x = startPose.position.x;
  double start_y = startPose.position.y;
  double start_z = startPose.position.z;
  obj->pose = {start_x, start_y, start_z, 0, 0, 0};
  while (!stopsign) {
    geometry_msgs::Pose currentPose = listener.getMarkerposition();
    double current_x = currentPose.position.x;
    double current_y = currentPose.position.y;
    double current_z = currentPose.position.z;
    using std::sqrt;
    using std::pow;
    obj->dimensions = {sqrt(pow(current_x - start_x, 2.0) +
                            pow(current_y - start_y, 2.0) +
                            pow(current_z - start_z, 2.0))};
    Q_EMIT
    dimensions_updated(obj->dimensions);
    collision_objects_mannager.add_collision_object(*obj, false);
  }
}

void ObjectDrawer::draw_cone(geometry_msgs::Pose startPose) {
  obj->shape = CONE;
  double start_x = startPose.position.x;
  double start_y = startPose.position.y;
  double start_z = startPose.position.z;
  obj->pose = {start_x, start_y, start_z, 0, 0, 0};
  while (!stopsign) {
    geometry_msgs::Pose currentPose = listener.getMarkerposition();
    double current_x = currentPose.position.x;
    double current_y = currentPose.position.y;
    double current_z = currentPose.position.z;
    using std::sqrt;
    using std::pow;
    obj->dimensions = {
        current_z - start_z,
        sqrt(pow(current_x - start_x, 2.0) + pow(current_y - start_y, 2.0))};
    obj->pose = {start_x, start_y, (current_z - start_z) / 2 + start_z, 0, 0,
                 0};
    Q_EMIT
    dimensions_updated(obj->dimensions);

    if (collision_objects_mannager.isvalid(*obj))
      collision_objects_mannager.add_collision_object(*obj, false);
  }
}

void ObjectDrawer::draw_cylinder(geometry_msgs::Pose startPose) {
  obj->shape = CYLINDER;
  double start_x = startPose.position.x;
  double start_y = startPose.position.y;
  double start_z = startPose.position.z;
  obj->pose = {start_x, start_y, start_z, 0, 0, 0};
  while (!stopsign) {
    geometry_msgs::Pose currentPose = listener.getMarkerposition();
    double current_x = currentPose.position.x;
    double current_y = currentPose.position.y;
    double current_z = currentPose.position.z;
    using std::sqrt;
    using std::pow;
    obj->pose = {start_x, start_y, (current_z - start_z) / 2 + start_z, 0, 0,
                 0};
    obj->dimensions = {
        current_z - start_z,
        sqrt(pow(current_x - start_x, 2.0) + pow(current_y - start_y, 2.0))};
    Q_EMIT
    dimensions_updated(obj->dimensions);
    collision_objects_mannager.add_collision_object(*obj, false);
  }
}

void ObjectDrawer::run() {
  QMutexLocker lock(&mutex);
  geometry_msgs::Pose startPose = listener.getMarkerposition();
  if (obj->shape == BOX) draw_box(startPose);
  if (obj->shape == SPHERE) draw_sphere(startPose);
  if (obj->shape == CONE) draw_cone(startPose);
  if (obj->shape == CYLINDER) draw_cylinder(startPose);
  return;
}

void ObjectDrawer::stop() { stopsign = true; }
