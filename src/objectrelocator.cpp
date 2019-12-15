#include "../include/move_ur5_qt/objectrelocator.hpp"
#include "../include/move_ur5_qt/globalobj.hpp"

ObjectRelocator::ObjectRelocator(){};
ObjectRelocator::~ObjectRelocator() {
  delete obj;
  obj = nullptr;
}

void ObjectRelocator::run() {
  QMutexLocker lock(&mutex);
  while (!stopsign) {
    geometry_msgs::Pose pose = listener.getMarkerposition();
    double x, y, z, qx, qy, qz, qw;
    x = pose.position.x;
    y = pose.position.y;
    z = pose.position.z;
    qx = pose.orientation.w;
    qy = pose.orientation.x;
    qz = pose.orientation.y;
    qw = pose.orientation.z;
    obj->pose = {x, y, z, qx, qy, qz, qw};
    Q_EMIT
    positionUpdated(obj->pose);
    collision_objects_mannager.add_collision_object(*obj, false);
  }
  return;
}

void ObjectRelocator::move_object_with_marker(collision_object* obj_ptr) {
  stopsign = false;
  obj = obj_ptr;
  start();
}

void ObjectRelocator::stop() { stopsign = true; }
