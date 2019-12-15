#ifndef PICKANDPLACEDIALOG_HPP
#define PICKANDPLACEDIALOG_HPP
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/transform_listener.h>
#endif
#include <QDialog>
#include "ui_pickandplacedialog.h"
#include "taskprogrammer.hpp"
#include "globalobj.hpp"

namespace Ui {
class PickAndPlaceDialog;
}

class PickAndPlaceDialog : public QDialog {
  Q_OBJECT

 public:
  explicit PickAndPlaceDialog(QWidget *parent = 0);
  ~PickAndPlaceDialog();

 private Q_SLOTS:
  void on_pushButton_setpick_clicked();

  void on_pushButton_setplace_clicked();

  void on_pushButton_Program_clicked();

  void on_checkBox_Repeat_toggled(bool checked);

  void on_pushButton_execute_clicked();

  void close();

 private:
  Ui::PickAndPlaceDialog *ui;
  TaskProgrammer taskprogrammer;
  geometry_msgs::Pose pickPose;
  geometry_msgs::Pose placePose;
};

#endif  // PICKANDPLACEDIALOG_HPP
