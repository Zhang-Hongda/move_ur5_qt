#include "../include/move_ur5_qt/pickandplacedialog.hpp"
#include <sstream>

std::string geopose2string(geometry_msgs::Pose pose) {
  std::stringstream ss;
  ss << pose.position.x << "," << pose.position.y << "," << pose.position.z
     << "," << pose.orientation.x << "," << pose.orientation.y << ","
     << pose.orientation.z << "," << pose.orientation.w;
  return ss.str();
}

PickAndPlaceDialog::PickAndPlaceDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::PickAndPlaceDialog) {
  ui->setupUi(this);
  QObject::connect(this, SIGNAL(accepted()), this, SLOT(close()));
}

PickAndPlaceDialog::~PickAndPlaceDialog() { delete ui; }

void PickAndPlaceDialog::on_pushButton_setpick_clicked() {
  pickPose = listener.getMarkerposition();
  std::string pickPose_str = geopose2string(pickPose);
  logger.log(Info, "Set pick point: [" + pickPose_str + "]");
  ui->lineEdit_pickpoint->setText(QString(pickPose_str.c_str()));
}

void PickAndPlaceDialog::on_pushButton_setplace_clicked() {
  placePose = listener.getMarkerposition();
  std::string placePose_str = geopose2string(placePose);
  logger.log(Info, "Set place point: [" + placePose_str + "]");
  ui->lineEdit_placepoint->setText(QString(placePose_str.c_str()));
}

void PickAndPlaceDialog::on_pushButton_Program_clicked() {
  taskprogrammer.program_pick_and_place_task(pickPose, placePose);
}

void PickAndPlaceDialog::on_checkBox_Repeat_toggled(bool checked) {}

void PickAndPlaceDialog::on_pushButton_execute_clicked() {
  taskprogrammer.execute_task();
}

void PickAndPlaceDialog::close() { taskprogrammer.clear_waypoints(); }
