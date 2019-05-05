#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QtDebug>
#include <QDebug>
#include <QString>
#include "../include/move_ur5_qt/main_window.hpp"
#include "../include/move_ur5_qt/qnode.hpp"
#include "../include/move_ur5_qt/tf_listener.hpp"
#include "../include/move_ur5_qt/timer.hpp"
#include "../include/move_ur5_qt/gesture_handler.hpp"
#include "../include/move_ur5_qt/trajectoryxmlwriter.hpp"
#include "../include/move_ur5_qt/trajectoryxmlreader.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

using namespace Qt;
extern QMutex m_mutex;
extern bool stopSign;
extern int frequency;

extern QMutex t_mutex;
extern bool istimeup;

extern QMutex G_mutex;
extern bool isEnable;

inline bool getisEnable() {
  QMutexLocker locker(&G_mutex);
  return isEnable;
}

inline void setisEnable(bool sign) {
  QMutexLocker locker(&G_mutex);
  isEnable = sign;
}

inline bool readstopSigen() {
  QMutexLocker locker(&m_mutex);
  return stopSign;
}

inline void setstopSign(bool sign) {
  QMutexLocker locker(&m_mutex);
  stopSign = sign;
}

inline int readfrequency() {
  QMutexLocker locker(&m_mutex);
  return frequency;
}

inline void setfrequency(int f) {
  QMutexLocker locker(&m_mutex);
  frequency = f;
}

inline bool getistimeup() {
  QMutexLocker locker(&t_mutex);
  return istimeup;
}

inline void setistimeup(bool sign) {
  QMutexLocker locker(&t_mutex);
  istimeup = sign;
}

inline double dist(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
  return sqrt(pow(pose1.position.x - pose2.position.x, 2) +
              pow(pose1.position.y - pose2.position.y, 2) +
              pow(pose1.position.z - pose2.position.z, 2));
}
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent),
      qnode(argc, argv),
      listener(),
      timer(),
      writer(),
      reader(),
      gesture_handler() {
  ui.setupUi(this);   // Calling this incidentally connects all ui's triggers to
                      // on_...() callbacks in this class.
  ros::Time::init();  // Initialize ros time first or it may crash
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  disableAllwidgets();                 // wait for ros thread

  /*********************
  ** Eable Gui
  **********************/
  QObject::connect(&qnode, SIGNAL(addObjectsFinished()), this,
                   SLOT(enableAllwidgets()));
  /*********************
  ** Shut Down
  **********************/
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Robot Info Logging
  **********************/
  ui.view_logging_RI->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateRIloggingView()));

  /*********************
  ** Scene Info Logging
  **********************/
  ui.view_logging_SI->setModel(listener.loggingModel());
  QObject::connect(&listener, SIGNAL(loggingUpdated()), this,
                   SLOT(updateSIloggingView()));
  /*********************
  ** Auto Start
  **********************/
  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
  /*********************
  ** Robot Status
  **********************/
  qRegisterMetaType<std::string>("std::string");
  ui.lineEdit_Orientation->setReadOnly(true);
  ui.lineEdit_Position->setReadOnly(true);
  ui.lineEdit_JointAngels->setReadOnly(true);
  QObject::connect(&qnode, SIGNAL(positionUpdated(std::string)), this,
                   SLOT(updatelineEdit_Position(std::string)));
  QObject::connect(&qnode, SIGNAL(orientationUpdated(std::string)), this,
                   SLOT(updatelineEdit_Orientation(std::string)));
  QObject::connect(&qnode, SIGNAL(jointvaluesUpdated(std::string)), this,
                   SLOT(updatelineEdit_Jointvalues(std::string)));

  /*********************
  ** Marker Status
  **********************/
  qRegisterMetaType<std::string>("std::string");
  ui.lineEdit_MO->setReadOnly(true);
  ui.lineEdit_MP->setReadOnly(true);
  QObject::connect(&listener, SIGNAL(positionUpdated(std::string)), this,
                   SLOT(updatelineEdit_MP(std::string)));
  QObject::connect(&listener, SIGNAL(orientationUpdated(std::string)), this,
                   SLOT(updatelineEdit_MO(std::string)));

  /*********************
  ** Tracking
  **********************/
  QObject::connect(this, SIGNAL(startTracking()), &listener,
                   SLOT(startTracking()));
  /*********************
  ** Recording
  **********************/
  qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");
  QObject::connect(&listener, SIGNAL(gotMarkerposition(geometry_msgs::Pose)),
                   this, SLOT(recordMarkerposition(geometry_msgs::Pose)));
  QObject::connect(&timer, SIGNAL(timeUp()), this, SLOT(timeUp()));
  QObject::connect(&timer, SIGNAL(timeNow(int)), this,
                   SLOT(updateprogressBar_T(int)));

  /*********************
  ** Plan
  **********************/
  QObject::connect(&qnode, SIGNAL(planningFinished(float)), this,
                   SLOT(update_progressBar_R(float)));
  /*********************
  ** HGR
  **********************/
  QObject::connect(&gesture_handler, SIGNAL(recordstart(bool)), ui.pushButton_R,
                   SLOT(setChecked(bool)));
  QObject::connect(&gesture_handler, SIGNAL(recordstop(bool)), ui.pushButton_R,
                   SLOT(setChecked(bool)));
  QObject::connect(&gesture_handler, SIGNAL(recordfinish(bool)),
                   ui.pushButton_F, SLOT(setChecked(bool)));
  QObject::connect(&gesture_handler, SIGNAL(recordrestart(bool)), this,
                   SLOT(on_gestureRestart_preformed(bool)));
  QObject::connect(&gesture_handler, SIGNAL(plan()), ui.pushButton_P,
                   SLOT(click()));
  QObject::connect(&gesture_handler, SIGNAL(execut()), ui.pushButton_E,
                   SLOT(click()));
  QObject::connect(&gesture_handler, SIGNAL(planandexecut()), ui.pushButton_PE,
                   SLOT(click()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::on_button_connect_clicked(bool check) {
  if (ui.checkbox_use_environment->isChecked()) {
    if (!qnode.init()) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      qnode.log(qnode.Info, "Connection Established.");
    }
  } else {
    if (!qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString())) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      qnode.log(qnode.Info, "Connection Established.");
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
// Log update
void MainWindow::updateRIloggingView() {
  if (ui.checkBox_RAC->isChecked()) {
    qnode.clearRobotstatusview(10);
  }
  ui.view_logging_RI->scrollToBottom();
}
void MainWindow::updateSIloggingView() {
  if (ui.checkBox_SAC->isChecked()) {
    listener.clearRobotstatusview(10);
  }
  ui.view_logging_SI->scrollToBottom();
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(
      this, tr("About ..."),
      tr("<h2>PACKAGE_NAME move_ur5_qt </h2><p>Copyright Eric Zhang </p><p>An "
         "application for robot trajectory programming by demonstration.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "move_ur5_qt");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url =
      settings.value("master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked_ev = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked_ev);
  if (checked_ev) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
  bool checked_ut = settings.value("use_timer", false).toBool();
  ui.checkBox_UT->setChecked(checked_ut);
  ui.spinBox_CD->setEnabled(checked_ut);
  ui.progressBar_T->setVisible(checked_ut);
  ui.spinBox_CD->setValue(settings.value("countDown", 5).toInt());
  QString path =
      settings.value("path", QString(
                                 "/home/eric/work_space/qt_ws/src/move_ur5_qt/"
                                 "data/trajectory.xml")).toString();
  ui.lineEdit_Path->setText(path);
  bool threshod = settings.value("threshod", false).toBool();
  ui.checkBox_Threshod->setChecked(threshod);
  ui.spinBox_Rate->setValue(settings.value("rate", 80).toInt());
  ui.checkBox_RAC->setChecked(settings.value("rl_autoclear", false).toBool());
  ui.checkBox_SAC->setChecked(settings.value("sl_autoclear", false).toBool());
  ui.spinBox_Fre->setValue(settings.value("frequency", 80).toInt());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "move_ur5_qt");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("use_timer", ui.checkBox_UT->isChecked());
  settings.setValue("countDown", ui.spinBox_CD->value());
  settings.setValue("path", ui.lineEdit_Path->text());
  settings.setValue("threshod", QVariant(ui.checkBox_Threshod->isChecked()));
  settings.setValue("rate", ui.spinBox_Rate->value());
  settings.setValue("rl_autoclear", QVariant(ui.checkBox_RAC->isChecked()));
  settings.setValue("sl_autoclear", QVariant(ui.checkBox_SAC->isChecked()));
  settings.setValue("frequency", ui.spinBox_Fre->value());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace move_ur5_qt

void move_ur5_qt::MainWindow::on_pushButton_ST_toggled(bool checked) {
  if (checked) {
    ui.tab_manager->setCurrentIndex(1);  // show second tab.
    setstopSign(false);
    setfrequency(ui.spinBox_Fre->value());
    ui.pushButton_F->setEnabled(true);
    Q_EMIT startTracking();
    ui.pushButton_ST->setText("Stop Tracking");
    //    ui.pushButton_ST->setEnabled(false);
  } else {
    setstopSign(true);
    ui.pushButton_ST->setText("Start Tracking");
    ui.pushButton_F->setEnabled(false);
    updatelineEdit_MP("");
    updatelineEdit_MO("");
  }
}

void move_ur5_qt::MainWindow::on_pushButton_MF_clicked() {
  qnode.move_Forward();
}

void move_ur5_qt::MainWindow::on_pushButtonMU_clicked() { qnode.move_Up(); }

void move_ur5_qt::MainWindow::on_pushButton_MH_clicked() { qnode.move_Home(); }

void move_ur5_qt::MainWindow::on_quit_button_clicked() {
  qnode.removeObjects();
}

void move_ur5_qt::MainWindow::updatelineEdit_Position(std::string position) {
  ui.lineEdit_Position->setText(QString::fromStdString(position));
}

void move_ur5_qt::MainWindow::updatelineEdit_Orientation(
    std::string orientation) {
  ui.lineEdit_Orientation->setText(QString::fromStdString(orientation));
}

void move_ur5_qt::MainWindow::updatelineEdit_Jointvalues(
    std::string jointvalues) {
  ui.lineEdit_JointAngels->setText(QString::fromStdString(jointvalues));
}

void move_ur5_qt::MainWindow::updatelineEdit_MP(std::string position) {
  ui.lineEdit_MP->setText(QString::fromStdString(position));
}

void move_ur5_qt::MainWindow::updatelineEdit_MO(std::string orientation) {
  ui.lineEdit_MO->setText(QString::fromStdString(orientation));
}

void move_ur5_qt::MainWindow::on_pushButton_CR_clicked() {
  qnode.clearRobotstatusview();
}

void move_ur5_qt::MainWindow::on_pushButton_CS_clicked() {
  listener.clearRobotstatusview();
}

void move_ur5_qt::MainWindow::on_pushButton_PO_clicked() { qnode.addObjects(); }

void move_ur5_qt::MainWindow::on_pushButton_CO_clicked() {
  qnode.removeObjects();
}

void move_ur5_qt::MainWindow::disableAllwidgets() {
  ui.groupBox_RPDcontrolPanel->setEnabled(false);
  ui.groupBox_Robotinfo->setEnabled(false);
  ui.groupBox_Robotstatus->setEnabled(false);
  ui.groupBox_Sceneinfo->setEnabled(false);
  ui.groupBox_Sceneobjects->setEnabled(false);
  ui.groupBox_Markerstatus->setEnabled(false);
  ui.groupBox_PE->setEnabled(false);
  ui.quit_button->setEnabled(false);
  ui.groupBox_SL->setEnabled(false);
}

void move_ur5_qt::MainWindow::enableAllwidgets() {
  ui.groupBox_RPDcontrolPanel->setEnabled(true);
  ui.groupBox_Robotinfo->setEnabled(true);
  ui.groupBox_Robotstatus->setEnabled(true);
  ui.groupBox_Sceneinfo->setEnabled(true);
  ui.groupBox_Sceneobjects->setEnabled(true);
  ui.groupBox_Markerstatus->setEnabled(true);
  ui.quit_button->setEnabled(true);
  ui.groupBox_SL->setEnabled(true);
  ui.pushButton_F->setEnabled(false);
}

void move_ur5_qt::MainWindow::recordMarkerposition(geometry_msgs::Pose pose) {
  if (ui.pushButton_R->isChecked()) {
    if (!waypoints.empty()) {
      if (dist(waypoints[waypoints.size() - 1], pose) < 0.01) {
        listener.log(listener.Fatal, "NO MOVEMENT DETECTED!");
        return;
      }
    }
    std::stringstream ss_p, ss_o;
    ss_p << "x: " << pose.position.x << " y: " << pose.position.y
         << " z: " << pose.position.z;
    ss_o << "qw: " << pose.orientation.w << "qx: " << pose.orientation.x
         << " qy: " << pose.orientation.y << " qz: " << pose.orientation.z;
    listener.log(listener.Info, "Received: " + ss_p.str() + " " + ss_o.str());
    ss_p.str("");
    ss_o.str("");
    waypoints.push_back(pose);
    qnode.publishMarkerposition(waypoints);
  }
}

void move_ur5_qt::MainWindow::timeUp() {
  ui.pushButton_R->setChecked(false);
  ui.pushButton_R->setEnabled(true);
  ui.checkBox_UT->setEnabled(true);
  ui.spinBox_CD->setEnabled(true);
}

void move_ur5_qt::MainWindow::updateprogressBar_T(int t) {
  ui.progressBar_T->setValue(t);
}

void move_ur5_qt::MainWindow::on_pushButton_R_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (checked) {
    setistimeup(false);
    ui.groupBox_PE->setEnabled(false);
    ui.pushButton_LF->setEnabled(false);
    ui.tab_manager->setCurrentIndex(1);  // show second tab.
    listener.log(listener.Info, "Recording Start");
    ui.pushButton_R->setText("Stop");
    ui.pushButton_S->setEnabled(true);
    if (ui.checkBox_UT->isChecked()) {
      ui.checkBox_UT->setEnabled(false);
      //      ui.pushButton_R->setEnabled(false);
      ui.spinBox_CD->setEnabled(false);
      int time = ui.spinBox_CD->value();
      ui.progressBar_T->setRange(0, time * 10);
      ui.progressBar_T->setValue(0);
      timer.init(time);
    }
  } else {
    setistimeup(true);
    timeUp();
    ui.pushButton_LF->setEnabled(true);
    listener.log(listener.Info, "Recording Stop");
    listener.log(listener.Info, "Received points number:  " +
                                    std::to_string(waypoints.size()));
    ui.pushButton_R->setText("Record");
    ui.checkBox_UT->setEnabled(true);
  }
}

void move_ur5_qt::MainWindow::on_checkBox_UT_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (checked) {
    listener.log(listener.Info, "Use timer.");
    ui.spinBox_CD->setEnabled(true);
    ui.progressBar_T->setVisible(true);
  } else {
    ui.spinBox_CD->setEnabled(false);
    ui.progressBar_T->setVisible(false);
    listener.log(listener.Info, "Disable timer.");
  }
}

void move_ur5_qt::MainWindow::on_pushButton_F_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (checked) {
  	ui.pushButton_R->setChecked(false);
    ui.groupBox_PE->setEnabled(true);
    ui.pushButton_E->setEnabled(false);
    ui.pushButton_R->setChecked(false);
    ui.pushButton_R->setEnabled(false);
    ui.pushButton_F->setText("Restart");
    setistimeup(true);
    listener.log(listener.Info,
                 "Total points:  " + std::to_string(waypoints.size()));
    qnode.publishMarkerposition(waypoints);
  } else {
    ui.progressBar_R->setValue(0);
    ui.progressBar_T->setValue(0);
    ui.groupBox_PE->setEnabled(false);
    ui.pushButton_R->setEnabled(true);
    ui.pushButton_F->setText("Finished");
    setistimeup(false);
    waypoints.clear();
    listener.log(listener.Info, "Restart! Waypoints Cleared!");
    qnode.publishMarkerposition(waypoints);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_P_clicked() {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  qnode.publishMarkerposition(waypoints);
  qnode.planTrajectory(waypoints);
  if (!ui.checkBox_Threshod->isChecked()) ui.pushButton_E->setEnabled(true);
}

void move_ur5_qt::MainWindow::update_progressBar_R(float rate) {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  ui.progressBar_R->setValue(rate);
  if (rate < ui.spinBox_Rate->value()) {
    qnode.log(qnode.Fatal, "FAILED!");
    ui.pushButton_E->setEnabled(false);
    ui.pushButton_PE->setEnabled(false);
  } else {
    qnode.log(qnode.Info, "SUCCESS!");
    ui.pushButton_E->setEnabled(true);
    ui.pushButton_PE->setEnabled(true);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_E_clicked() {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  qnode.executeTrajectory(waypoints);
}

void move_ur5_qt::MainWindow::on_pushButton_S_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  QString path = ui.lineEdit_Path->text();
  listener.log(listener.Info,
               "Total points:  " + std::to_string(waypoints.size()));
  if (path.isEmpty()) {
    path = QFileDialog::getSaveFileName(this, tr("Save File"), ".",
                                        tr("XML Files(*.xml)"));
  }
  if (writer.writeFile(waypoints, path)) {
    ui.lineEdit_Path->setText(path);
    listener.log(listener.Info, "SUCCESS! File saved to " + path.toStdString());
  } else {
    QMessageBox::critical(this, tr("Error"), tr("Invalid path %1").arg(path));
    listener.log(listener.Fatal, "FAILED! File not saved!");
  }
}

void move_ur5_qt::MainWindow::on_checkBox_Threshod_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  qnode.log(qnode.Info, "Use threshod.");
  if (checked) {
    ui.spinBox_Rate->setEnabled(true);
  } else {
    ui.spinBox_Rate->setEnabled(false);
    qnode.log(qnode.Info, "Disable threshod.");
  }
}

void move_ur5_qt::MainWindow::on_spinBox_Rate_editingFinished() {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  qnode.log(qnode.Info,
            "Set threshod: " + std::to_string(ui.spinBox_Rate->value()));
}

void move_ur5_qt::MainWindow::on_lineEdit_Path_editingFinished() {
  ui.tab_manager->setCurrentIndex(0);  // show first tab.
  qnode.log(qnode.Info, "Set path: " + ui.lineEdit_Path->text().toStdString());
}

void move_ur5_qt::MainWindow::on_spinBox_CD_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  listener.log(listener.Info,
               "Set timer: " + std::to_string(ui.spinBox_CD->value()));
}

void move_ur5_qt::MainWindow::on_spinBox_Fre_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  setfrequency(ui.spinBox_Fre->value());
  listener.log(listener.Info,
               "Set frequency: " + std::to_string(ui.spinBox_Fre->value()));
}

void move_ur5_qt::MainWindow::on_pushButton_LF_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (!waypoints.empty()) {
    if (QMessageBox::Yes ==
        QMessageBox::question(this, tr("Warning"),
                              tr("Previous points will be deleted. Continue?"),
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::Yes)) {
      waypoints.clear();
    } else {
      listener.log(listener.Fatal, "FAILED! File not load!");
      return;
    }
  }
  ui.groupBox_PE->setEnabled(true);
  if (reader.readFile(waypoints)) {
    listener.log(listener.Info, "SUCCESS! File loaded. ");
    listener.log(listener.Info,
                 "Total points:  " + std::to_string(waypoints.size()));
    qnode.publishMarkerposition(waypoints);
  } else {
    listener.log(listener.Fatal, "FAILED! File not load!");
    return;
  }
}

void move_ur5_qt::MainWindow::on_pushButton_EGC_toggled(bool checked) {
  if (checked) {
    ui.pushButton_EGC->setText("Disable Gesture Control");
    setisEnable(true);
    gesture_handler.init();
  } else {
    setisEnable(false);
    ui.pushButton_EGC->setText("Enable Gesture Control");
  }
}

void move_ur5_qt::MainWindow::on_pushButton_PE_clicked() {
  ui.pushButton_P->click();
  // on_pushButton_P_clicked();
  ui.pushButton_E->click();
  // on_pushButton_E_clicked();
}

void move_ur5_qt::MainWindow::on_gestureRestart_preformed(bool checked) {
	if(ui.pushButton_R->isChecked()){
		ui.pushButton_R->setChecked(false);
		ui.pushButton_F->setChecked(true);
		ui.pushButton_F->setChecked(false);
	}else{
		ui.pushButton_F->setChecked(true);
		ui.pushButton_F->setChecked(false);
	}
	if(ui.pushButton_F->isChecked()){
		ui.pushButton_F->setChecked(false);
	}
}
