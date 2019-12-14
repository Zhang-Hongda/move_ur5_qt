/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <sstream>
#include "../include/move_ur5_qt/tf_listener.hpp"

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
QMutex m_mutex;

Tf_listener::Tf_listener() {}

Tf_listener::~Tf_listener() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void Tf_listener::init() {
  listener = std::make_shared<tf::TransformListener>();
  return;
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
  logger.log(Info, "Tracking Start");
  stopsign = false;
  start();
}
// Stop the thread
void Tf_listener::stopTracking() {
  logger.log(Info, "Tracking Stop");
  stopsign = true;
}
// Get pose
void Tf_listener::run() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  tf::StampedTransform transform;
  while (ros::ok()) {
    QMutexLocker locker(&m_mutex);
    if (stopsign) break;
    try {
      ros::Time now = ros::Time(0);
      listener->waitForTransform(base_frame, target_frame, now,
                                 ros::Duration(0.5));
      listener->lookupTransform(base_frame, target_frame, now, transform);
      marker_pose = transform2pose(transform);
      updatePositon(marker_pose);
      updateOrientation(marker_pose);
      Q_EMIT gotMarkerposition(marker_pose);
    } catch (tf::TransformException &ex) {
      logger.log(Info, ex.what());
    }
    ros::Duration(1.0 / frequency).sleep();
  }
  return;
}

// Emit pose info
geometry_msgs::Pose Tf_listener::getMarkerposition() { return marker_pose; }

// Set freq
void Tf_listener::setfrequency(int freq) {
  frequency = freq;
  logger.log(Info, "Set tracking frequency: %d", frequency);
  return;
}

// Set frame
void Tf_listener::setmarkerframe(std::string frame) {
  target_frame = frame;
  logger.log(Info, "Set target frame: " + target_frame);
}

void Tf_listener::setbaseframe(std::string frame) {
  base_frame = frame;
  logger.log(Info, "Set base frame: " + base_frame);
}
