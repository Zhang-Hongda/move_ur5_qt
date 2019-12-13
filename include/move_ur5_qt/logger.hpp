#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include "globaldata.h"

class Logger : public QObject {
  Q_OBJECT
 public:
  explicit Logger(QObject *parent = nullptr);
  void log(const LogLevel &level, const std::string &msg);

signals:
  void loggingUpdated();
 public slots:
};

#endif  // LOGGER_H
