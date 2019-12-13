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
** Namespaces
*****************************************************************************/
namespace move_ur5_qt {
extern QMutex m_mutex;
extern bool stopSign;
extern int frequency;
/*****************************************************************************
** Class
*****************************************************************************/
class Tf_listener : public QThread {
  Q_OBJECT
 public:
  Tf_listener();
  virtual ~Tf_listener();
  bool init();
  void run();
  geometry_msgs::Pose getMarkerposition();
  void updatePositon(geometry_msgs::Pose pose);
  void updateOrientation(geometry_msgs::Pose pose);

 public Q_SLOTS:
  void startTracking();
  void stopTracking();
  void setfrequency(int freq);

Q_SIGNALS:
  void positionUpdated(std::string position);
  void orientationUpdated(std::string orientation);
  void recordingFinished(std::vector<geometry_msgs::Pose> waypoints);
  void gotMarkerposition(geometry_msgs::Pose pose);

 private:
  std::shared_ptr<tf::TransformListener> listener;
  std::shared_ptr<ros::NodeHandle> node_handle;
  geometry_msgs::Pose marker_pose;
  int frequency;
  bool stopsign;
};

}  // namespace move_ur5_qt

#endif  // TF_LISTENER_H
