#include <time.h>
#include <QThread>

#include "../include/move_ur5_qt/gesture_handler.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

QMutex G_mutex;
bool isEnable = false;

inline bool getisEnable() {
  QMutexLocker locker(&G_mutex);
  return isEnable;
}

inline void setisEnable(bool sign) {
  QMutexLocker locker(&G_mutex);
  isEnable = sign;
}

/*****************************************************************************
** Implementation
*****************************************************************************/
Gesture_handler::Gesture_handler() {}

Gesture_handler::~Gesture_handler() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool Gesture_handler::init() {
  if (!ros::isInitialized()) {
    return false;
  }
  ros::start();  // explicitly needed
  ros::NodeHandle node_handle;
  gesture_sub =
      node_handle.subscribe("gesture", 1, &Gesture_handler::callback, this);
  start();
  return true;
}

void Gesture_handler::callback(const std_msgs::Int8::ConstPtr& msg) {
  if (!getisEnable()) return;
  switch (msg->data) {
    case 0:
      break;
    case 1: {
      Q_EMIT
      recordstart(true);
      break;
    }
    case 2: {
      Q_EMIT
      recordstop(false);
      break;
    }
    case 3: {
      Q_EMIT
      recordfinish(true);
      break;
    }
    case 4: {
      Q_EMIT
      recordrestart(false);
      break;
    }
    case 5: {
      Q_EMIT
      plan();
      break;
    }
    case 6: {
      Q_EMIT
      execut();
      break;
    }
    case 7: {
      Q_EMIT
      planandexecut();
      break;
    }
  }
}

void Gesture_handler::run() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
}

}  // namespace move_ur5_qt
