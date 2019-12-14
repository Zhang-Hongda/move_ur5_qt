#ifndef TF_LISTENER_H
#define TF_LISTENER_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/transform_listener.h>
#endif
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <math.h>
#include <time.h>
#include <QThread>
#include <QMutex>
#include <QStringListModel>
#include "globalobj.hpp"
Q_DECLARE_METATYPE(std::vector<geometry_msgs::Pose>)
Q_DECLARE_METATYPE(geometry_msgs::Pose)

/*****************************************************************************
** Class
*****************************************************************************/
extern QMutex m_mutex;

class Tf_listener : public QThread {
  Q_OBJECT
 public:
  Tf_listener();
  virtual ~Tf_listener();
  void init();
  void run();
  geometry_msgs::Pose getMarkerposition();
  void updatePositon(geometry_msgs::Pose pose);
  void updateOrientation(geometry_msgs::Pose pose);

 public Q_SLOTS:
  void startTracking();
  void stopTracking();
  void setfrequency(int freq);
  void setmarkerframe(std::string frame);
  void setbaseframe(std::string frame);

Q_SIGNALS:
  void positionUpdated(std::string position);
  void orientationUpdated(std::string orientation);
  void recordingFinished(std::vector<geometry_msgs::Pose> waypoints);
  void gotMarkerposition(geometry_msgs::Pose pose);

 private:
  std::shared_ptr<tf::TransformListener> listener;
  geometry_msgs::Pose marker_pose;
  int frequency = 1;
  bool stopsign;
  std::string target_frame = "/marker";
  std::string base_frame = "/base_link";

 public:
  //  QMutex m_mutex;
};

#endif  // TF_LISTENER_H
