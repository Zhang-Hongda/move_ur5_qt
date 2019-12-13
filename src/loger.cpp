#include "logger.hpp"
#include <sstream>

Logger::Logger(QObject *parent) : QObject(parent) {}

void Logger::log(const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
      logging_model_msg << "[DEBUG] ["
                        << "]: " << msg;
      break;
    }
    case (Info): {
      logging_model_msg << "[INFO] ["
                        << "]: " << msg;
      break;
    }
    case (Warn): {
      logging_model_msg << "[INFO] ["
                        << "]: " << msg;
      break;
    }
    case (Error): {
      logging_model_msg << "[ERROR] ["
                        << "]: " << msg;
      break;
    }
    case (Fatal): {
      logging_model_msg << "[FATAL] ["
                        << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}
