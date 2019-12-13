/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef move_ur5_qt_QNODE_HPP_
#define move_ur5_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#endif
#include <string>
#include <math.h>
#include <time.h>
#include <QThread>
#include <QMutex>
#include <QStringListModel>
#include "globalobj.hpp"
Q_DECLARE_METATYPE(std::string)
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
  void move_Forward();
  void move_Up();
  void move_Home();
  void updatePositon();
  void updateOrientation();
  void updateJointvalues();
  void planTrajectory(std::vector<geometry_msgs::Pose> waypoints);
  void executeTrajectory(std::vector<geometry_msgs::Pose> waypoints);
  void publishMarkerposition(std::vector<geometry_msgs::Pose> waypoints);
Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void positionUpdated(std::string position);
  void orientationUpdated(std::string orientation);
  void jointvaluesUpdated(std::string jointvalues);
  void planningFinished(float rate);
  void executionFinished();

 private:
  int init_argc;
  char **init_argv;
  const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterfacePtr move_group;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::Publisher marker_pub;
  std::shared_ptr<ros::NodeHandle> node_handle;
};

}  // namespace move_ur5_qt

#endif /* move_ur5_qt_QNODE_HPP_ */
