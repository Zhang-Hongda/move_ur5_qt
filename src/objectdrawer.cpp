#include "../include/move_ur5_qt/objectdrawer.hpp"
#include "../include/move_ur5_qt/globalobj.hpp"

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

void ObjectDrawer::run() {
  QMutexLocker lock(&mutex);
  geometry_msgs::Pose startPose = listener.getMarkerposition();
  if (obj->shape == BOX) draw_box(startPose);
  return;
}

void ObjectDrawer::stop() { stopsign = true; }
