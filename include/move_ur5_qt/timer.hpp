#ifndef TIMER_HPP
#define TIMER_HPP
#include <time.h>
#include <QThread>
#include <QMutex>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {
extern QMutex t_mutex;
extern bool istimeup;
/*****************************************************************************
** Class
*****************************************************************************/
class Timer : public QThread {
  Q_OBJECT
 public:
  Timer();
  virtual ~Timer();
  void init(int t);
  void run();
Q_SIGNALS:
  void timeNow(int t);
  void timeUp();

 private:
  int time;
};
}  // namespace move_ur5_qt
#endif  // TIMER_HPP
