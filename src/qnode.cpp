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

namespace move_ur5_qt
{
/*****************************************************************************
** Implementation
*****************************************************************************/
inline std::vector<moveit_msgs::CollisionObject>
addTable_Shlef(moveit::planning_interface::MoveGroupInterface &move_group)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  moveit_msgs::CollisionObject object_table;
  object_table.header.frame_id = move_group.getPlanningFrame();
  object_table.id = "table";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = -0.02;
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = cos((M_PI) / 8);
  table_pose.orientation.z = sin((M_PI) / 8);
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.01;

  object_table.primitives.push_back(primitive);
  object_table.primitive_poses.push_back(table_pose);
  object_table.operation = object_table.ADD;
  collision_objects.push_back(object_table);

  moveit_msgs::CollisionObject object_shelf;
  object_shelf.header.frame_id = move_group.getPlanningFrame();
  object_shelf.id = "shelf";
  double width = 0.04, height = 1.02;
  primitive.dimensions[0] = width;
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = height;
  geometry_msgs::Pose shelf_pose;
  shelf_pose.orientation.w = cos((M_PI) / 8);
  shelf_pose.orientation.z = sin((M_PI) / 8);
  shelf_pose.position.x = 0.31;
  shelf_pose.position.y = -0.56;
  shelf_pose.position.z = height / 2;
  object_shelf.primitives.push_back(primitive);
  object_shelf.primitive_poses.push_back(shelf_pose);  // left

  primitive.dimensions[0] = 0.96;  // top
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = width;
  object_shelf.primitives.push_back(primitive);
  shelf_pose.position.x = 0.31 + 0.96 / 2 * cos(M_PI / 4);
  shelf_pose.position.y = -0.56 + 0.96 / 2 * sin(M_PI / 4);
  shelf_pose.position.z = height - width / 2;
  object_shelf.primitive_poses.push_back(shelf_pose);

  primitive.dimensions[0] = width;
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = height;
  shelf_pose.position.x = 0.31 + 0.96 * cos(M_PI / 4);
  shelf_pose.position.y = -0.56 + 0.96 * sin(M_PI / 4);
  shelf_pose.position.z = height / 2;
  object_shelf.primitives.push_back(primitive);
  object_shelf.primitive_poses.push_back(shelf_pose);  // right

  object_shelf.operation = object_shelf.ADD;
  collision_objects.push_back(object_shelf);
  return collision_objects;
}
inline bool moveToNamedTarget(moveit::planning_interface::MoveGroupInterface &move_group, std::string name)
{
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget(name);
  if (move_group.move())
  {
    return true;
  }
  else
  {
    return false;
  }
}
inline moveit_msgs::ObjectColor setColor(std::string name, float r, float g, float b, float a = 0.9f)
{
  moveit_msgs::ObjectColor color;
  color.id = name;
  color.color.r = r;
  color.color.g = g;
  color.color.b = b;
  color.color.a = a;
  return color;
}
inline geometry_msgs::Pose transform2pose(tf::StampedTransform transform)
{
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

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

// Init ros node
bool QNode::init()
{
  ros::init(init_argc, init_argv, "move_ur5_qt");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed
  node_handle = std::make_shared<ros::NodeHandle>("~");
  planning_scene_diff_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  marker_pub = node_handle->advertise<visualization_msgs::Marker>("visualization_marker", 1);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "move_ur5_qt");

  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed
  node_handle = std::make_shared<ros::NodeHandle>("~");
  planning_scene_diff_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  marker_pub = node_handle->advertise<visualization_msgs::Marker>("visualization_marker", 1);

  start();
  return true;
}

// Tracking robot state and update the information
void QNode::run()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::string EndEffectorLink;
  node_handle->param<std::string>("EndEffectorLink", EndEffectorLink, "tool0");
  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
  move_group->setPoseReferenceFrame("base_link");
  move_group->setEndEffectorLink(EndEffectorLink);
  move_group->allowReplanning(true);
  move_group->setGoalPositionTolerance(0.01);
  move_group->setGoalOrientationTolerance(0.05);
  planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  sleep(3);  // wait for planning scene interface initialization
  objects_id.push_back("table");
  objects_id.push_back("shelf");
  collision_objects = addTable_Shlef(*move_group);
  p.is_diff = true;
  p.object_colors.push_back(setColor("table", 0.8f, 0, 0, 0.5f));
  p.object_colors.push_back(setColor("shelf", 0, 0, 0.5f, 0.5f));
  addObjects();
  Q_EMIT
  addObjectsFinished();
  log(Info, "Tracking robot status.");
  while (ros::ok())
  {
    updatePositon();
    updateOrientation();
    updateJointvalues();
  }
  removeObjects();
  log(Info, "Ros shutdown, proceeding to close the gui.");
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

// Publish line
void QNode::publishMarkerposition(std::vector<geometry_msgs::Pose> waypoints)
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "/base_link";
  line_strip.header.stamp = ros::Time::now();
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.id = 0;
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;
  line_strip.scale.x = 0.01;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  for (std::vector<geometry_msgs::Pose>::const_iterator it = waypoints.begin(); it != waypoints.end(); it++)
  {
    geometry_msgs::Point p;
    p.x = it->position.x;
    p.y = it->position.y;
    p.z = it->position.z;
    line_strip.points.push_back(p);
  }
  marker_pub.publish(line_strip);
}

// Move robot to a named target
void QNode::move_Forward()
{
  if (moveToNamedTarget(*move_group, "forward"))
    log(Info, "Move Forward: SUCCEEDED");
  else
    log(Info, "Move Forward: FAILED");
}

void QNode::move_Up()
{
  if (moveToNamedTarget(*move_group, "up"))
    log(Info, "Move Up: SUCCEEDED");
  else
    log(Info, "Move Up: FAILED");
}

void QNode::move_Home()
{
  if (moveToNamedTarget(*move_group, "home"))
    log(Info, "Move Home: SUCCEEDED");
  else
    log(Info, "Move Home: FAILED");
}

// Clear robot status log
void QNode::clearRobotstatusview(int except)
{
  logging_model.removeRows(0, logging_model.rowCount() - except);
}

// Add or remove cllision objects
void QNode::addObjects()
{
  log(Info, "Add objects into the world");
  planning_scene_interface->addCollisionObjects(collision_objects);
  planning_scene_diff_publisher.publish(p);
}

void QNode::removeObjects()
{
  log(Info, "Remove objects");
  planning_scene_interface->removeCollisionObjects(objects_id);
}

// Update robot status
void QNode::updatePositon()
{
  geometry_msgs::Pose pose = move_group->getCurrentPose().pose;
  std::stringstream ss;
  ss << "x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z;
  Q_EMIT
  positionUpdated(ss.str());
  ss.str("");
}

void QNode::updateOrientation()
{
  geometry_msgs::Pose pose = move_group->getCurrentPose().pose;
  std::stringstream ss;
  ss << "qx: " << pose.orientation.x << " qy: " << pose.orientation.y << " qz: " << pose.orientation.z
     << " qw: " << pose.orientation.w;
  Q_EMIT
  orientationUpdated(ss.str());
  ss.str("");
}

void QNode::updateJointvalues()
{
  std::vector<double> jointvalues = move_group->getCurrentJointValues();
  std::stringstream ss;
  for (std::vector<double>::iterator it = jointvalues.begin(); it != jointvalues.end(); it++)
    ss << *it << " ";
  Q_EMIT
  jointvaluesUpdated(ss.str());
  ss.str("");
}

// Plan trajectory
void QNode::planTrajectory(std::vector<geometry_msgs::Pose> waypoints)
{
  log(Info, "Planning...");
  move_group->setMaxVelocityScalingFactor(0.5);  // jonit speed control
  const double jump_threshold = 0.0;
  const double eef_step = 0.005;
  double rate = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  plan.trajectory_ = trajectory;
  Q_EMIT planningFinished(rate * 100);
  log(Info, "Planning Finished.");
  log(Info, "Completed: " + std::to_string(rate * 100) + "%");
}

// Execute the plan
void QNode::executeTrajectory(std::vector<geometry_msgs::Pose> waypoints)
{
  if (0 == waypoints.size() || plan.trajectory_.joint_trajectory.points.empty())
  {
    log(Fatal, "Invalid plan!");
    Q_EMIT executionFinished();
    return;
  }
  moveit::planning_interface::MoveGroupInterface::Plan temp_plan;
  moveit::planning_interface::MoveItErrorCode success;
  std::vector<double> group_variable_values;
  move_group->getCurrentState()->copyJointGroupPositions(
      move_group->getCurrentState()->getRobotModel()->getJointModelGroup(move_group->getName()), group_variable_values);
  log(Info, "Executing plan.");
  if (!move_group->execute(plan))
  {
    log(Fatal, "Failed to execute the plan!");
    Q_EMIT executionFinished();
    return;
  }
  else
  {
    log(Info, "SUCCESS!");
  }

  log(Info, "Moving back to the start state.");
  move_group->setStartStateToCurrentState();
  move_group->setJointValueTarget(group_variable_values);
  success = move_group->plan(temp_plan);
  if (success)
  {
    if (!move_group->execute(temp_plan))
    {
      log(Fatal, "Failed to move back to the start state!");
      Q_EMIT executionFinished();
      return;
    }
    else
    {
      log(Info, "SUCCESS!");
    }
  }
  else
  {
    log(Fatal, "Can not find a path to the start state!");
    Q_EMIT executionFinished();
    return;
  }
  Q_EMIT executionFinished();
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

}  // namespace move_ur5_qt
