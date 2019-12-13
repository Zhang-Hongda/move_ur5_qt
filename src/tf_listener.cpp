/*****************************************************************************
** Includes
*****************************************************************************/
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QtDebug>
#include <sstream>
#include <string>

#include "../include/move_ur5_qt/tf_listener.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {
QMutex m_mutex;
bool stopSign = true;
int frequency = 10;

inline bool readstopSigen() {
  QMutexLocker locker(&m_mutex);
  return stopSign;
}

inline void setstopSign(bool sign) {
  QMutexLocker locker(&m_mutex);
  stopSign = sign;
}

inline int readfrequency() {
  QMutexLocker locker(&m_mutex);
  return frequency;
}

inline void setfrequency(int f) {
  QMutexLocker locker(&m_mutex);
  frequency = f;
}

inline geometry_msgs::Pose transform2pose(tf::StampedTransform transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.getOrigin().getX();
  pose.position.y = transform.getOrigin().getY();
  pose.position.z = transform.getOrigin().getZ();
  pose.orientation.w = transform.getRotation().getW();
  pose.orientation.x = transform.getRotation().getX();
  pose.orientation.y = transform.getRotation().getY();
  pose.orientation.z = transform.getRotation().getZ();
  return pose;
}
/*****************************************************************************
** Implementation
*****************************************************************************/

Tf_listener::Tf_listener() {}

Tf_listener::~Tf_listener() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool Tf_listener::init() {
  if (!ros::isInitialized()) {
    return false;
  }
  ros::start();  // explicitly needed
  node_handle = std::make_shared<ros::NodeHandle>();
  listener = std::make_shared<tf::TransformListener>();
  return true;
}

// Emit pose info
void Tf_listener::updatePositon(geometry_msgs::Pose pose) {
  std::stringstream ss;
  ss << "x: " << pose.position.x << " y: " << pose.position.y
     << " z: " << pose.position.z;
  Q_EMIT
  positionUpdated(ss.str());
  ss.str("");
}

void Tf_listener::updateOrientation(geometry_msgs::Pose pose) {
  std::stringstream ss;
  ss << "qx: " << pose.orientation.x << " qy: " << pose.orientation.y
     << " qz: " << pose.orientation.z << " qw: " << pose.orientation.w;
  Q_EMIT
  orientationUpdated(ss.str());
  ss.str("");
}

// Start the thread
void Tf_listener::startTracking() {
  stopsign = false;
  init();
  start();
}
// Stop the thread
void Tf_listener::stopTracking() { stopsign = true; }
// Get pose
void Tf_listener::run() {
  logger.log(Info, "Tracking Start");
  QMutexLocker locker(&m_mutex);
  tf::StampedTransform transform;
  while (ros::ok() && !stopsign) {
    try {
      ros::Time now = ros::Time(0);
      std::string target_frame;
      node_handle->param<std::string>("target_frame", target_frame, "/marker");
      listener->waitForTransform("/base_link", target_frame, now,
                                 ros::Duration(0.5));
      listener->lookupTransform("/base_link", target_frame, now, transform);
      marker_pose = transform2pose(transform);
      updatePositon(marker_pose);
      updateOrientation(marker_pose);
      Q_EMIT gotMarkerposition(marker_pose);
    } catch (tf::TransformException &ex) {
      logger.log(Info, ex.what());
    }
    ros::Duration(1.0 / frequency).sleep();
  }
  logger.log(Info, "Tracking Stop");
  return;
}

// Emit pose info
geometry_msgs::Pose Tf_listener::getMarkerposition() { return marker_pose; }

// Set freq
void Tf_listener::setfrequency(int freq) {
  frequency = freq;
  return;
}
}
