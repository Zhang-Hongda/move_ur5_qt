#ifndef GESTURE_HANDLER_HPP
#define GESTURE_HANDLER_HPP
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#endif
#include <time.h>
#include <QThread>
#include <QMutex>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {
extern QMutex G_mutex;
extern bool isEnable;
/*****************************************************************************
** Class
*****************************************************************************/
class Gesture_handler : public QThread {
  Q_OBJECT
 public:
  Gesture_handler();
  virtual ~Gesture_handler();
  bool init();
  void run();
  void callback(const std_msgs::Int8::ConstPtr& msg);
Q_SIGNALS:
  void recordstart(bool sign);
  void recordstop(bool sign);
  void recordfinish(bool sign);
  void recordrestart(bool sign);
  void plan();
  void execut();
  void planandexecut();

 private:
  ros::Subscriber gesture_sub;
};
}  // namespace move_ur5_qt
#endif  // GESTURE_HANDLER_HPP
