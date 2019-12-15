#ifndef OBJECTRELOCATOR_HPP
#define OBJECTRELOCATOR_HPP
#include "globaldata.hpp"

#include <QThread>
#include <QMutex>

class ObjectRelocator : public QThread {
  Q_OBJECT
 public:
  ObjectRelocator();
  virtual ~ObjectRelocator();
  void move_object_with_marker(collision_object* obj_ptr);
  void stop();
  void run();
Q_SIGNALS:
  void positionUpdated(std::vector<double> pose);

 private:
  bool stopsign = true;
  collision_object* obj;
  QMutex mutex;
};

#endif  // OBJECTRELOCATOR_HPP
