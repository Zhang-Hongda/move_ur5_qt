#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include "globaldata.hpp"

class Logger : public QObject {
  Q_OBJECT
 public:
  explicit Logger(QObject *parent = nullptr);
  void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  void loggingUpdated();
};

#endif  // LOGGER_H
