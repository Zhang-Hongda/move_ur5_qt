#include <time.h>
#include <QThread>

#include "../include/move_ur5_qt/timer.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

QMutex t_mutex;
bool istimeup = false;
inline bool getistimeup() {
  QMutexLocker locker(&t_mutex);
  return istimeup;
}

inline void setistimeup(bool sign) {
  QMutexLocker locker(&t_mutex);
  istimeup = sign;
}

/*****************************************************************************
** Implementation
*****************************************************************************/
Timer::Timer() {}

Timer::~Timer() { wait(); }

void Timer::init(int t) {
  time = t;
  start();
}

void Timer::run() {
  for (int i = 0; i < time * 10; i++) {
    if (!getistimeup()) {
      msleep(100);
      Q_EMIT timeNow(i + 1);
    } else {
      return;
    }
  }
  Q_EMIT timeUp();
}
}  // namespace move_ur5_qt
