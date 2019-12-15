#ifndef TASKPROGRAMMER_HPP
#define TASKPROGRAMMER_HPP

#include "globaldata.hpp"
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/transform_listener.h>
#endif
#include <QThread>
#include <QMutex>

class TaskProgrammer : public QThread {
  Q_OBJECT
 public:
  TaskProgrammer();
  virtual ~TaskProgrammer();
  void program_pick_and_place_task(geometry_msgs::Pose pickPose,
                                   geometry_msgs::Pose placePose);
  void stop();
  void run();
  void execute_task();
  void clear_waypoints();
Q_SIGNALS:

 private:
  bool stopsign = true;
  QMutex mutex;
};

#endif  // TASKPROGRAMMER_HPP
