#ifndef COMANNAGERDIALOG_HPP
#define COMANNAGERDIALOG_HPP

#include <QDialog>
#include <QMutex>
#include "ui_comannagerdialog.h"
#include "objectrelocator.hpp"
#include "objectdrawer.hpp"
#include "globalobj.hpp"
Q_DECLARE_METATYPE(std::vector<double>)

namespace Ui {
class COMannagerDialog;
}

class COMannagerDialog : public QDialog {
  Q_OBJECT

 public:
  explicit COMannagerDialog(QWidget *parent = 0);
  ~COMannagerDialog();
  void ReadSettings();
  void WriteSettings();
  void closeEvent(QCloseEvent *);
 private Q_SLOTS:
  void on_lineEdit_frame_editingFinished();

  void on_lineEdit_name_editingFinished();

  void on_lineEdit_color_editingFinished();

  void on_lineEdit_dimensions_editingFinished();

  void on_lineEdit_pose_editingFinished();

  void on_pushButton_addobject_clicked();

  void on_comboBox_type_currentIndexChanged(const QString &arg1);

  void on_pushButton_markerpose_toggled(bool checked);

  void on_pushButton_draw_toggled(bool checked);

  void set_create_object_lineEdit_readOnly(bool);
  void on_pushButton_quit_clicked();

  void on_tabWidget_currentChanged(int index);

  void on_lineEdit_pose2_editingFinished();

  void on_pushButton_markerPose2_toggled(bool checked);

 public Q_SLOTS:
  void updatePosition(std::vector<double> pose);
  void updateDimension(std::vector<double> dims);

 private:
  Ui::COMannagerDialog *ui;
  collision_object temp_co;
  ObjectRelocator obj_relocator;
  ObjectDrawer obj_drawer;
};

#endif  // COMANNAGERDIALOG_HPP
