#ifndef GLOBALDATA_H
#define GLOBALDATA_H
#include <QStringListModel>
#include <vector>
#include <string>
#include <map>
using std::vector;
using std::string;
using std::map;
// logging
extern QStringListModel logging_model;
enum LogLevel { Debug, Info, Warn, Error, Fatal };
// basic shape
enum primitive_shape_set {
  BOX = 1u,
  SPHERE = 2u,
  CYLINDER = 3u,
  CONE = 4u,
};
extern map<string, primitive_shape_set> shape_map;
// all properties of a co
struct collision_object {
  string frame;
  string name;
  primitive_shape_set shape;
  vector<float> color;
  vector<double> dimensions;
  vector<double> pose;
  collision_object() {}
  collision_object(string _frame, string _name, primitive_shape_set _shape,
                   vector<float> _color, vector<double> _dimensions,
                   vector<double> _pose) {
    frame = _frame;
    name = _name;
    shape = _shape;
    color = _color;
    dimensions = _dimensions;
    pose = _pose;
  }
};

#endif  // GLOBALDATA_H
