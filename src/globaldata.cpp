#include "../include/move_ur5_qt/globaldata.hpp"
QStringListModel logging_model;
map<string, primitive_shape_set> shape_map{
    {"BOX", BOX}, {"SPHERE", SPHERE}, {"CYLINDER", CYLINDER}, {"CONE", CONE}};
std::vector<geometry_msgs::Pose> waypoints;
