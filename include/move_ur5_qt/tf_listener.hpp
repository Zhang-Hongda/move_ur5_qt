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
  enum LogLevel { Debug, Info, Warn, Error, Fatal };  // Logging
  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);
  void clearRobotstatusview(int except = 0);
  void getMarkerposition();
  void updatePositon(geometry_msgs::Pose pose);
  void updateOrientation(geometry_msgs::Pose pose);

 public Q_SLOTS:
  void startTracking();

Q_SIGNALS:
  void loggingUpdated();
  void positionUpdated(std::string position);
  void orientationUpdated(std::string orientation);
  void recordingFinished(std::vector<geometry_msgs::Pose> waypoints);
  void gotMarkerposition(geometry_msgs::Pose pose);

 private:
  QStringListModel logging_model;
  std::shared_ptr<tf::TransformListener> listener;
  std::shared_ptr<ros::NodeHandle> node_handle;
};

}  // namespace move_ur5_qt

#endif  // TF_LISTENER_H
