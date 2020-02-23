#ifndef OBJECTDRAWER_HPP
#define OBJECTDRAWER_HPP
#include "globaldata.hpp"
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/transform_listener.h>
#endif
#include <QThread>
#include <QMutex>

class ObjectDrawer : public QThread {
  Q_OBJECT
 public:
  ObjectDrawer();
  virtual ~ObjectDrawer();
  void run();
  void draw_object(collision_object* obj);
  void draw_box(geometry_msgs::Pose stratPose);
  void draw_sphere(geometry_msgs::Pose stratPose);
  void draw_cone(geometry_msgs::Pose stratPose);
  void draw_cylinder(geometry_msgs::Pose stratPose);
  void stop();
Q_SIGNALS:
  void dimensions_updated(std::vector<double>);

 private:
  QMutex mutex;
  bool stopsign = true;
  collision_object* obj;
};

#endif  // OBJECTDRAWER_HPP
