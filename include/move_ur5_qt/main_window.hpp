#ifndef move_ur5_qt_MAIN_WINDOW_H
#define move_ur5_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "tf_listener.hpp"
#include "timer.hpp"
#include "gesture_handler.hpp"
#include "trajectoryxmlwriter.hpp"
#include "trajectoryxmlreader.hpp"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();   // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing
  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();


 public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);
  /******************************************
  ** Manual connections
  *******************************************/
  void updateRIloggingView();
  void updateSIloggingView();
  void updatelineEdit_Position(std::string position);  // update robot status
  void updatelineEdit_Orientation(std::string orientation);
  void updatelineEdit_Jointvalues(std::string jointvalues);
  void updatelineEdit_MP(std::string position);  // update marker status
  void updatelineEdit_MO(std::string orientation);
  void recordMarkerposition(geometry_msgs::Pose pose);
  void disableAllwidgets();
  void enableAllwidgets();
  void timeUp();
  void updateprogressBar_T(int t);
  void update_progressBar_R(float rate);
  void on_gestureRestart_preformed(bool);

  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
 private Q_SLOTS:
  void on_pushButton_ST_toggled(bool checked);
  void on_pushButton_MF_clicked();
  void on_quit_button_clicked();
  void on_pushButtonMU_clicked();
  void on_pushButton_MH_clicked();
  void on_pushButton_CR_clicked();
  void on_pushButton_PO_clicked();
  void on_pushButton_CO_clicked();
  void on_pushButton_CS_clicked();
  void on_pushButton_R_toggled(bool checked);
  void on_checkBox_UT_toggled(bool checked);
  void on_pushButton_F_toggled(bool checked);
  void on_pushButton_P_clicked();
  void on_pushButton_E_clicked();
  void on_pushButton_S_clicked();
  void on_checkBox_Threshod_toggled(bool checked);
  void on_spinBox_Rate_editingFinished();
  void on_lineEdit_Path_editingFinished();
  void on_spinBox_CD_editingFinished();
  void on_spinBox_Fre_editingFinished();

  void on_pushButton_LF_clicked();

  void on_pushButton_EGC_toggled(bool checked);

  void on_pushButton_PE_clicked();

Q_SIGNALS:
  void startTracking();

 private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  Tf_listener listener;
  Timer timer;
  Gesture_handler gesture_handler;
  std::vector<geometry_msgs::Pose> waypoints;
  trajectoryXMLWriter writer;
  trajectoryXMLReader reader;
};

}  // namespace move_ur5_qt

#endif  // move_ur5_qt_MAIN_WINDOW_H
