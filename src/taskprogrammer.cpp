#include "../include/move_ur5_qt/taskprogrammer.hpp"
#include "../include/move_ur5_qt/globalobj.hpp"

TaskProgrammer::TaskProgrammer(){};
TaskProgrammer::~TaskProgrammer() {}

void TaskProgrammer::run() {}
void TaskProgrammer::stop() {}

void TaskProgrammer::program_pick_and_place_task(
    geometry_msgs::Pose pickPose, geometry_msgs::Pose placePose) {
  logger.log(Info, "Plan pick-and-place task.");
  double delta = 0.20;
  geometry_msgs::Pose waypoint_1 = pickPose;
  waypoint_1.position.z += delta;
  geometry_msgs::Pose waypoint_2 = placePose;
  waypoint_2.position.z += delta;
  waypoints = {pickPose, waypoint_1, waypoint_2, placePose};
  qnode.publishMarkerposition(waypoints);
  qnode.planTrajectory(waypoints);
}

void TaskProgrammer::execute_task() {
  logger.log(Info, "Execute pick-and-place task.");
  qnode.executeTrajectory(waypoints);
}

void TaskProgrammer::clear_waypoints() {
  logger.log(Info, "Clear waypoints.");
  waypoints.clear();
  qnode.publishMarkerposition(waypoints);
}
