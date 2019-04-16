/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/String.h>
#include <sstream>
#include <QtDebug>
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

// Cleat log info except the bottom line
void Tf_listener::clearRobotstatusview(int except) {
  logging_model.removeRows(0, logging_model.rowCount() - except);
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
  init();
  start();
}

// Get pose
void Tf_listener::run() {
  log(Info, "Tracking Start");
  getMarkerposition();
  log(Info, "Tracking Stop");
}

// Emit pose info
void Tf_listener::getMarkerposition() {
  tf::StampedTransform transform;
  geometry_msgs::Pose temp_pose;
  while (ros::ok() && !readstopSigen()) {
    try {
      ros::Time now = ros::Time(0);
      std::string target_frame;
      node_handle->param<std::string>("target_frame", target_frame, "/marker");
      listener->waitForTransform("/base_link", target_frame, now,
                                 ros::Duration(0.5));
      listener->lookupTransform("/base_link", target_frame, now, transform);
      temp_pose = transform2pose(transform);
      updatePositon(temp_pose);
      updateOrientation(temp_pose);
      Q_EMIT gotMarkerposition(temp_pose);
    } catch (tf::TransformException &ex) {
      log(Info, ex.what());
    }
    ros::Duration(1.0 / readfrequency()).sleep();
  }
}

// Sence log
void Tf_listener::log(const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info): {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn): {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error): {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal): {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}
}
