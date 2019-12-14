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
      init_argc(argc),
      init_argv(argv),
      qnode(),
      timer(),
      writer(),
      reader(),
      gesture_handler(),
      collision_objects_mannager(argc, argv) {
  ui.setupUi(this);   // Calling this incidentally connects all ui's triggers to
                      // on_...() callbacks in this class.
  ros::Time::init();  // Initialize ros time first or it may crash
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  disableAllwidgets();                 // wait for ros thread

  /*********************
  ** Logging
  **********************/
  ui.listView_log->setModel(&logging_model);
  QObject::connect(&logger, SIGNAL(loggingUpdated()), this,
                   SLOT(updateloggingView()));

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
                   SLOT(gestureRestart_preformed(bool)));
  QObject::connect(&gesture_handler, SIGNAL(plan()), ui.pushButton_P,
                   SLOT(click()));
  QObject::connect(&gesture_handler, SIGNAL(execut()), ui.pushButton_E,
                   SLOT(click()));
  QObject::connect(&gesture_handler, SIGNAL(planandexecut()), ui.pushButton_PE,
                   SLOT(click()));

  /*********************
  ** Collision objects mannager
  **********************/
  QObject::connect(&collision_objects_mannager,
                   SIGNAL(Collision_Objects_Updated()), this,
                   SLOT(update_collision_objects_list()));
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

void MainWindow::on_button_connect_clicked() {
  if (ui.checkbox_use_environment->isChecked()) {
    ros::init(init_argc, init_argv, "move_ur5_qt_node");
  } else {
    std::string master_url = ui.line_edit_master->text().toStdString();
    std::string host_url = ui.line_edit_host->text().toStdString();
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "move_ur5_qt_node");
  }
  if (!ros::master::check()) {
    showNoMasterMessage();
    return;
  }
  ros::start();  // explicitly needed
  qnode.init();
  qnode.setParent(this);
  listener.init();
  listener.setParent(this);
  collision_objects_mannager.init();
  collision_objects_mannager.setParent(this);
  logger.log(Info, "Connection Established.");
  enableAllwidgets();
  ui.line_edit_master->setEnabled(false);
  ui.line_edit_host->setEnabled(false);
  ReadSettingsAfterStartup();
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

void MainWindow::updateloggingView() {
  if (ui.checkBox_autoclearlog->isChecked()) {
    logger.clearlog(15);
  }
  ui.listView_log->scrollToBottom();
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
      settings.value("master_url", QString("http://eric-inspiron-7560:11311"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("127.0.0.1")).toString();
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
}

void MainWindow::ReadSettingsAfterStartup() {
  if (!ui.checkbox_remember_settings->isChecked()) return;
  QSettings settings("Qt-Ros Package", "move_ur5_qt");
  bool checked_ut =
      settings.value("use_timer", false).toBool();  // use timer or not
  ui.checkBox_UT->setChecked(checked_ut);
  ui.spinBox_CD->setEnabled(checked_ut);
  ui.progressBar_T->setEnabled(checked_ut);
  ui.spinBox_CD->setValue(
      settings.value("countDown", 5).toInt());  // countdown value
  // save trajectory file path
  QString path =
      settings.value("path", QString(
                                 "/home/eric/work_space/qt_ws/src/move_ur5_qt/"
                                 "data/trajectory.xml")).toString();
  ui.lineEdit_Path->setText(path);
  bool threshod =
      settings.value("threshod", false).toBool();  // use threshod or not
  ui.checkBox_Threshod->setChecked(threshod);
  ui.spinBox_Rate->setValue(
      settings.value("rate", 80).toInt());  // threshold rate
  ui.checkBox_autoclearlog->setChecked(
      settings.value("log_autoclear", false).toBool());
  ui.spinBox_Fre->setValue(
      settings.value("frequency", 10).toInt());  // sample freq
  // objects auto load
  QString objpath =
      settings.value("object_file_path",
                     QString(
                         "/home/eric/work_space/workspace_ros/new_ws/src/"
                         "pcl_tracker/collision_objects")).toString();
  ui.lineEdit_objpath->setText(objpath);
  ui.checkBox_autoload->setChecked(settings.value("auto_load", false).toBool());
  ui.lineEdit_markerframename->setText(
      settings.value("marker_frame", QString("/marker")).toString());
  ui.lineEdit_baseframename->setText(
      settings.value("base_frame", QString("/base_link")).toString());
  on_lineEdit_markerframename_editingFinished();
  on_lineEdit_baseframename_editingFinished();
  listener.setfrequency(ui.spinBox_Fre->value());
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
  settings.setValue("log_autoclear",
                    QVariant(ui.checkBox_autoclearlog->isChecked()));
  settings.setValue("frequency", ui.spinBox_Fre->value());
  settings.setValue(
      "object_file_path",
      ui.lineEdit_objpath->text());  // auto load collision object file path
  settings.setValue("auto_load", ui.checkBox_autoload->isChecked());
  settings.setValue("marker_frame", ui.lineEdit_markerframename->text());
  settings.setValue("base_frame", ui.lineEdit_baseframename->text());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace move_ur5_qt

void move_ur5_qt::MainWindow::on_pushButton_ST_toggled(bool checked) {
  if (checked) {
    ui.tab_manager->setCurrentIndex(1);  // show second tab.
    listener.startTracking();
    ui.pushButton_F->setEnabled(true);
    ui.pushButton_ST->setText("Stop Tracking");
  } else {
    listener.stopTracking();
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
  on_pushButton_clearall_clicked();
  qApp->quit();
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

void move_ur5_qt::MainWindow::disableAllwidgets() {
  ui.groupBox_RPDcontrolPanel->setEnabled(false);
  ui.groupBox_log->setEnabled(false);
  ui.groupBox_Robotstatus->setEnabled(false);
  ui.groupBox_Sceneobjects->setEnabled(false);
  ui.groupBox_Markerstatus->setEnabled(false);
  ui.groupBox_PE->setEnabled(false);
  ui.quit_button->setEnabled(false);
  ui.groupBox_SL->setEnabled(false);
  ui.groupBox_hgr->setEnabled(false);
  ui.groupBox_basiccontrol->setEnabled(false);
}

void move_ur5_qt::MainWindow::enableAllwidgets() {
  ui.groupBox_RPDcontrolPanel->setEnabled(true);
  ui.groupBox_log->setEnabled(true);
  ui.groupBox_Robotstatus->setEnabled(true);
  ui.groupBox_Sceneobjects->setEnabled(true);
  ui.groupBox_Markerstatus->setEnabled(true);
  ui.quit_button->setEnabled(true);
  ui.groupBox_SL->setEnabled(true);
  ui.pushButton_F->setEnabled(false);
  ui.groupBox_hgr->setEnabled(true);
  ui.groupBox_basiccontrol->setEnabled(true);
  ui.button_connect->setEnabled(false);
}

void move_ur5_qt::MainWindow::recordMarkerposition(geometry_msgs::Pose pose) {
  if (ui.pushButton_R->isChecked()) {
    if (!waypoints.empty()) {
      if (dist(waypoints[waypoints.size() - 1], pose) < 0.01) {
        logger.log(Fatal, "NO MOVEMENT DETECTED!");
        return;
      }
    }
    std::stringstream ss_p, ss_o;
    ss_p << "x: " << pose.position.x << " y: " << pose.position.y
         << " z: " << pose.position.z;
    ss_o << "qw: " << pose.orientation.w << "qx: " << pose.orientation.x
         << " qy: " << pose.orientation.y << " qz: " << pose.orientation.z;
    logger.log(Info, "Received: " + ss_p.str() + " " + ss_o.str());
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
    logger.log(Info, "Recording Start");
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
    logger.log(Info, "Recording Stop");
    logger.log(Info,
               "Received points number:  " + std::to_string(waypoints.size()));
    ui.pushButton_R->setText("Record");
    ui.checkBox_UT->setEnabled(true);
  }
}

void move_ur5_qt::MainWindow::on_checkBox_UT_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (checked) {
    logger.log(Info, "Use timer.");
    ui.spinBox_CD->setEnabled(true);
    ui.progressBar_T->setEnabled(true);
  } else {
    ui.spinBox_CD->setEnabled(false);
    ui.progressBar_T->setEnabled(false);
    logger.log(Info, "Disable timer.");
  }
}

void move_ur5_qt::MainWindow::on_pushButton_F_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  if (checked) {
    ui.groupBox_PE->setEnabled(true);
    ui.pushButton_E->setEnabled(false);
    ui.pushButton_R->setChecked(false);
    ui.pushButton_R->setEnabled(false);
    ui.pushButton_F->setText("Restart");
    setistimeup(true);
    logger.log(Info, "Total points:  " + std::to_string(waypoints.size()));
    qnode.publishMarkerposition(waypoints);
  } else {
    ui.progressBar_R->setValue(0);
    ui.progressBar_T->setValue(0);
    ui.groupBox_PE->setEnabled(false);
    ui.pushButton_R->setEnabled(true);
    ui.pushButton_F->setText("Finished");
    setistimeup(false);
    waypoints.clear();
    logger.log(Info, "Restart! Waypoints Cleared!");
    qnode.publishMarkerposition(waypoints);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_P_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  qnode.publishMarkerposition(waypoints);
  qnode.planTrajectory(waypoints);
  if (!ui.checkBox_Threshod->isChecked()) ui.pushButton_E->setEnabled(true);
}

void move_ur5_qt::MainWindow::update_progressBar_R(float rate) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  ui.progressBar_R->setValue(rate);
  if (rate < ui.spinBox_Rate->value()) {
    logger.log(Info, "FAILED!");
    ui.pushButton_E->setEnabled(false);
    ui.pushButton_PE->setEnabled(false);
  } else {
    logger.log(Info, "SUCCESS!");
    ui.pushButton_E->setEnabled(true);
    ui.pushButton_PE->setEnabled(true);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_E_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  qnode.executeTrajectory(waypoints);
}

void move_ur5_qt::MainWindow::on_pushButton_S_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  QString path = ui.lineEdit_Path->text();
  logger.log(Info, "Total points:  " + std::to_string(waypoints.size()));
  if (path.isEmpty()) {
    path = QFileDialog::getSaveFileName(this, tr("Save File"), ".",
                                        tr("XML Files(*.xml)"));
  }
  if (writer.writeFile(waypoints, path)) {
    ui.lineEdit_Path->setText(path);
    logger.log(Info, "SUCCESS! File saved to " + path.toStdString());
  } else {
    QMessageBox::critical(this, tr("Error"), tr("Invalid path %1").arg(path));
    logger.log(Fatal, "FAILED! File not saved!");
  }
}

void move_ur5_qt::MainWindow::on_checkBox_Threshod_toggled(bool checked) {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  logger.log(Info, "Use threshod.");
  if (checked) {
    ui.spinBox_Rate->setEnabled(true);
  } else {
    ui.spinBox_Rate->setEnabled(false);
    logger.log(Info, "Disable threshod.");
  }
}

void move_ur5_qt::MainWindow::on_spinBox_Rate_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  logger.log(Info, "Set threshod: " + std::to_string(ui.spinBox_Rate->value()));
}

void move_ur5_qt::MainWindow::on_lineEdit_Path_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  logger.log(Info, "Set path: " + ui.lineEdit_Path->text().toStdString());
}

void move_ur5_qt::MainWindow::on_spinBox_CD_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  logger.log(Info, "Set timer: " + std::to_string(ui.spinBox_CD->value()));
}

void move_ur5_qt::MainWindow::on_spinBox_Fre_editingFinished() {
  ui.tab_manager->setCurrentIndex(1);  // show sec tab.
  listener.setfrequency(ui.spinBox_Fre->value());
}

void move_ur5_qt::MainWindow::on_pushButton_LF_clicked() {
  ui.tab_manager->setCurrentIndex(1);  // show first tab.
  ui.groupBox_PE->setEnabled(false);
  if (!waypoints.empty()) {
    if (QMessageBox::Yes ==
        QMessageBox::question(this, tr("Warning"),
                              tr("Previous points will be deleted. Continue?"),
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::Yes)) {
      waypoints.clear();
    } else {
      logger.log(Fatal, "FAILED! File not load!");
      return;
    }
  }
  if (reader.readFile(waypoints)) {
    logger.log(Info, "SUCCESS! File loaded. ");
    logger.log(Info, "Total points:  " + std::to_string(waypoints.size()));
    qnode.publishMarkerposition(waypoints);
  } else {
    logger.log(Fatal, "FAILED! File not load!");
    return;
  }
  if (!waypoints.empty()) ui.groupBox_PE->setEnabled(true);
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

void move_ur5_qt::MainWindow::gestureRestart_preformed(bool checked) {
  if (ui.pushButton_R->isChecked()) {
    ui.pushButton_R->setChecked(false);
    ui.pushButton_F->setChecked(true);
    ui.pushButton_F->setChecked(false);
  } else {
    ui.pushButton_F->setChecked(true);
    ui.pushButton_F->setChecked(false);
  }
  if (ui.pushButton_F->isChecked()) {
    ui.pushButton_F->setChecked(false);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_LO_clicked() {
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), ".",
                                                  tr("TXT Files(*.txt)"));
  if (collision_objects_mannager.load_collision_objects_form_file(
          fileName.toStdString())) {
    collision_objects_mannager.update();
  }
}

// automatically load collision objects from a folder
void move_ur5_qt::MainWindow::on_checkBox_autoload_toggled(bool checked) {
  if (checked) {
    string dir = ui.lineEdit_objpath->text().toStdString();
    if (dir.empty()) {
      QString path = QFileDialog::getExistingDirectory(this, tr("Find Folder"),
                                                       QDir::currentPath());
      ui.lineEdit_objpath->setText(path);
      dir = path.toStdString();
    }
    collision_objects_mannager.load_collision_objects_form_dir(dir);
    collision_objects_mannager.update();
    update_collision_objects_list();
  }
}

// update co list
void move_ur5_qt::MainWindow::update_collision_objects_list() {
  ui.listWidget_objlist->clear();
  for (string id : collision_objects_mannager.get_objects_names()) {
    QListWidgetItem *item = new QListWidgetItem;
    item->setText(id.c_str());
    item->setCheckState(Qt::Unchecked);
    ui.listWidget_objlist->addItem(item);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_selectall_clicked() {
  int cnt = ui.listWidget_objlist->count();
  for (int i = 0; i < cnt; ++i) {
    QListWidgetItem *item = ui.listWidget_objlist->item(i);
    item->setCheckState(Qt::Checked);
  }
}

void move_ur5_qt::MainWindow::on_pushButton_clearselected_clicked() {
  std::vector<QListWidgetItem *> selected;
  int cnt = ui.listWidget_objlist->count();
  for (int i = 0; i < cnt; ++i) {
    QListWidgetItem *item = ui.listWidget_objlist->item(i);
    if (item->checkState() == Qt::Checked) {
      selected.push_back(item);
    }
  }
  for (QListWidgetItem *item : selected) {
    collision_objects_mannager.remove_object(item->text().toStdString());
    int row = ui.listWidget_objlist->row(item);
    ui.listWidget_objlist->takeItem(row);
    delete item;
  }
  collision_objects_mannager.update();
}

void move_ur5_qt::MainWindow::on_pushButton_clearall_clicked() {
  on_pushButton_selectall_clicked();
  on_pushButton_clearselected_clicked();
  logger.log(Info, "All collision objects cleared.");
}

void move_ur5_qt::MainWindow::on_pushButton_clearlog_clicked() {
  logger.clearlog();
}

void move_ur5_qt::MainWindow::on_lineEdit_markerframename_editingFinished() {
  listener.setmarkerframe(ui.lineEdit_markerframename->text().toStdString());
}

void move_ur5_qt::MainWindow::on_lineEdit_baseframename_editingFinished() {
  listener.setbaseframe(ui.lineEdit_baseframename->text().toStdString());
}
