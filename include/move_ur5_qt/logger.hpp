#ifndef LOGGER_H
#define LOGGER_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QObject>
#include "globaldata.hpp"
#include "stringbuffer.hpp"

class Logger : public QObject {
  Q_OBJECT
 public:
  explicit Logger(QObject *parent = nullptr);
  //  void log(const LogLevel &level, const std::string &msg);
  template <typename... Args>
  void log(const LogLevel &level, const char *format, Args... args) {
    std::string msg = StringBuffer::Format(format, args...);
    logging_model.insertRows(logging_model.rowCount(), 1);
    std::stringstream logging_model_msg;
    switch (level) {
      case (Debug): {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
      }
      case (Info): {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
      }
      case (Warn): {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
      }
      case (Error): {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
      }
      case (Fatal): {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
      }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                          new_row);
    Q_EMIT loggingUpdated();  // used to readjust the scrollbar
  }

  void log(const LogLevel &level, const std::string &msg);
  void clearlog(int except = 0);
Q_SIGNALS:
  void loggingUpdated();
};

#endif  // LOGGER_H
