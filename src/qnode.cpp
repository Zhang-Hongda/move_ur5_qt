/*****************************************************************************
** Includes
*****************************************************************************/
#include <math.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <QtDebug>
#include <sstream>
#include <string>
#include "../include/move_ur5_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {
/*****************************************************************************
** Implementation
*****************************************************************************/

inline bool moveToNamedTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    std::string name) {
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget(name);
  if (move_group.move()) {
    return true;
  } else {
    return false;
  }
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

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

// Init ros node
bool QNode::init() {
  ros::init(init_argc, init_argv, "move_ur5_qt");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed
  node_handle = std::make_shared<ros::NodeHandle>();
  planning_scene_diff_publisher =
      node_handle->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  marker_pub = node_handle->advertise<visualization_msgs::Marker>(
      "visualization_marker", 1);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "move_ur5_qt");

  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed
  node_handle = std::make_shared<ros::NodeHandle>();
  planning_scene_diff_publisher =
      node_handle->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  marker_pub = node_handle->advertise<visualization_msgs::Marker>(
      "visualization_marker", 1);

  start();
  return true;
}

// Tracking robot state and update the information
void QNode::run() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::string EndEffectorLink;
  node_handle->param<std::string>("EndEffectorLink", EndEffectorLink, "tool0");
  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      PLANNING_GROUP);
  move_group->setPoseReferenceFrame("base_link");
  move_group->setEndEffectorLink(EndEffectorLink);
  move_group->allowReplanning(true);
  move_group->setGoalPositionTolerance(0.01);
  move_group->setGoalOrientationTolerance(0.05);

  log(Info, "Tracking robot status.");
  while (ros::ok()) {
    updatePositon();
    updateOrientation();
    updateJointvalues();
  }
  log(Info, "Ros shutdown, proceeding to close the gui.");
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

// Publish line
void QNode::publishMarkerposition(std::vector<geometry_msgs::Pose> waypoints) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "/base_link";
  line_strip.header.stamp = ros::Time::now();
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.id = 0;
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;
  line_strip.scale.x = 0.01;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  for (std::vector<geometry_msgs::Pose>::const_iterator it = waypoints.begin();
       it != waypoints.end(); it++) {
    geometry_msgs::Point p;
    p.x = it->position.x;
    p.y = it->position.y;
    p.z = it->position.z;
    line_strip.points.push_back(p);
  }
  marker_pub.publish(line_strip);
}

// Move robot to a named target
void QNode::move_Forward() {
  if (moveToNamedTarget(*move_group, "forward"))
    log(Info, "Move Forward: SUCCEEDED");
  else
    log(Info, "Move Forward: FAILED");
}

void QNode::move_Up() {
  if (moveToNamedTarget(*move_group, "up"))
    log(Info, "Move Up: SUCCEEDED");
  else
    log(Info, "Move Up: FAILED");
}

void QNode::move_Home() {
  if (moveToNamedTarget(*move_group, "home"))
    log(Info, "Move Home: SUCCEEDED");
  else
    log(Info, "Move Home: FAILED");
}

// Clear robot status log
void QNode::clearRobotstatusview(int except) {
  logging_model.removeRows(0, logging_model.rowCount() - except);
}

// Update robot status
void QNode::updatePositon() {
  geometry_msgs::Pose pose = move_group->getCurrentPose().pose;
  std::stringstream ss;
  ss << "x: " << pose.position.x << " y: " << pose.position.y
     << " z: " << pose.position.z;
  Q_EMIT
  positionUpdated(ss.str());
  ss.str("");
}

void QNode::updateOrientation() {
  geometry_msgs::Pose pose = move_group->getCurrentPose().pose;
  std::stringstream ss;
  ss << "qx: " << pose.orientation.x << " qy: " << pose.orientation.y
     << " qz: " << pose.orientation.z << " qw: " << pose.orientation.w;
  Q_EMIT
  orientationUpdated(ss.str());
  ss.str("");
}

void QNode::updateJointvalues() {
  std::vector<double> jointvalues = move_group->getCurrentJointValues();
  std::stringstream ss;
  for (std::vector<double>::iterator it = jointvalues.begin();
       it != jointvalues.end(); it++)
    ss << *it << " ";
  Q_EMIT
  jointvaluesUpdated(ss.str());
  ss.str("");
}

// Plan trajectory
void QNode::planTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
  log(Info, "Planning...");
  move_group->setMaxVelocityScalingFactor(0.5);  // jonit speed control
  const double jump_threshold = 0.0;
  const double eef_step = 0.005;
  double rate = move_group->computeCartesianPath(waypoints, eef_step,
                                                 jump_threshold, trajectory);
  plan.trajectory_ = trajectory;
  Q_EMIT planningFinished(rate * 100);
  log(Info, "Planning Finished.");
  log(Info, "Completed: " + std::to_string(rate * 100) + "%");
}

// Execute the plan
void QNode::executeTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
  if (0 == waypoints.size() ||
      plan.trajectory_.joint_trajectory.points.empty()) {
    log(Fatal, "Invalid plan!");
    Q_EMIT executionFinished();
    return;
  }
  moveit::planning_interface::MoveGroupInterface::Plan temp_plan;
  moveit::planning_interface::MoveItErrorCode success;
  std::vector<double> group_variable_values;
  move_group->getCurrentState()->copyJointGroupPositions(
      move_group->getCurrentState()->getRobotModel()->getJointModelGroup(
          move_group->getName()),
      group_variable_values);
  log(Info, "Executing plan.");
  if (!move_group->execute(plan)) {
    log(Fatal, "Failed to execute the plan!");
    Q_EMIT executionFinished();
    return;
  } else {
    log(Info, "SUCCESS!");
  }

  log(Info, "Moving back to the start state.");
  move_group->setStartStateToCurrentState();
  move_group->setJointValueTarget(group_variable_values);
  success = move_group->plan(temp_plan);
  if (success) {
    if (!move_group->execute(temp_plan)) {
      log(Fatal, "Failed to move back to the start state!");
      Q_EMIT executionFinished();
      return;
    } else {
      log(Info, "SUCCESS!");
    }
  } else {
    log(Fatal, "Can not find a path to the start state!");
    Q_EMIT executionFinished();
    return;
  }
  Q_EMIT executionFinished();
}

void QNode::log(const LogLevel &level, const std::string &msg) {
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

}  // namespace move_ur5_qt
