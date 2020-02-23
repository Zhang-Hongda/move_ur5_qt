#include "../include/move_ur5_qt/comannagerdialog.hpp"
#include <QSettings>

COMannagerDialog::COMannagerDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::COMannagerDialog) {
  ui->setupUi(this);
  ReadSettings();
  qRegisterMetaType<std::vector<double>>("std::vector<double>");
  QObject::connect(&obj_relocator, SIGNAL(positionUpdated(std::vector<double>)),
                   this, SLOT(updatePosition(std::vector<double>)));
  QObject::connect(&obj_drawer, SIGNAL(dimensions_updated(std::vector<double>)),
                   this, SLOT(updateDimension(std::vector<double>)));
}

COMannagerDialog::~COMannagerDialog() { delete ui; }

void COMannagerDialog::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QDialog::closeEvent(event);
}

void COMannagerDialog::WriteSettings() {
  QSettings settings("ChildWindow", "co_mannager");
  settings.setValue("obj_name", ui->lineEdit_name->text());
  settings.setValue("obj_frame", ui->lineEdit_frame->text());
  settings.setValue("obj_color", ui->lineEdit_color->text());
  settings.setValue("obj_pose", ui->lineEdit_pose->text());
  settings.setValue("obj_dims", ui->lineEdit_dimensions->text());
  settings.setValue("obj_type", ui->comboBox_type->currentIndex());
}

void COMannagerDialog::ReadSettings() {
  QSettings settings("ChildWindow", "co_mannager");
  restoreGeometry(settings.value("ChildWindow_geometry").toByteArray());
  QString obj_name = settings.value("obj_name", QString("obj")).toString();
  QString obj_frame =
      settings.value("obj_frame", QString("base_link")).toString();
  QString obj_color =
      settings.value("obj_color", QString("255,0,0")).toString();
  QString obj_pose =
      settings.value("obj_pose", QString("0,0,0,0,0,0")).toString();
  QString obj_dims = settings.value("obj_dims", QString("0.1")).toString();
  ui->lineEdit_name->setText(obj_name);
  ui->lineEdit_frame->setText(obj_frame);
  ui->lineEdit_color->setText(obj_color);
  ui->lineEdit_pose->setText(obj_pose);
  ui->lineEdit_dimensions->setText(obj_dims);
  int type_index = settings.value("obj_type", 2).toInt();
  ui->comboBox_type->setCurrentIndex(type_index);
  on_lineEdit_frame_editingFinished();
  on_lineEdit_name_editingFinished();
  on_comboBox_type_currentIndexChanged(ui->comboBox_type->currentText());
  on_lineEdit_color_editingFinished();
  on_lineEdit_dimensions_editingFinished();
  on_lineEdit_pose_editingFinished();
}

void COMannagerDialog::on_lineEdit_frame_editingFinished() {
  temp_co.frame = ui->lineEdit_frame->text().toStdString();
}

void COMannagerDialog::on_lineEdit_name_editingFinished() {
  temp_co.name = ui->lineEdit_name->text().toStdString();
}

void COMannagerDialog::on_comboBox_type_currentIndexChanged(
    const QString &arg1) {
  temp_co.shape = shape_map[arg1.toStdString()];
}

void COMannagerDialog::on_lineEdit_color_editingFinished() {
  temp_co.color = string2vector<float>(
      "[" + ui->lineEdit_color->text().toStdString() + "]");
}

void COMannagerDialog::on_lineEdit_dimensions_editingFinished() {
  temp_co.dimensions = string2vector<double>(
      "[" + ui->lineEdit_dimensions->text().toStdString() + "]");
}

void COMannagerDialog::on_lineEdit_pose_editingFinished() {
  temp_co.pose = string2vector<double>(
      "[" + ui->lineEdit_pose->text().toStdString() + "]");
}

void COMannagerDialog::on_pushButton_addobject_clicked() {
  collision_objects_mannager.add_collision_object(temp_co);
}

void COMannagerDialog::set_create_object_lineEdit_readOnly(bool flag) {
  ui->lineEdit_frame->setReadOnly(flag);
  ui->lineEdit_name->setReadOnly(flag);
  ui->lineEdit_color->setReadOnly(flag);
  ui->lineEdit_dimensions->setReadOnly(flag);
  ui->lineEdit_pose->setReadOnly(flag);
  ui->groupBox_controlpanel->setEnabled(!flag);
  ui->comboBox_type->setEnabled(!flag);  // disable type selection
}

void COMannagerDialog::on_pushButton_markerpose_toggled(bool checked) {
  ui->pushButton_draw->setChecked(false);
  if (checked) {
    ui->pushButton_quit->setEnabled(false);
    logger.log(Info, "Moving object with marker: START.");
    set_create_object_lineEdit_readOnly(true);
    ui->pushButton_draw->setEnabled(false);
    if (collision_objects_mannager.isvalid(temp_co)) {
      obj_relocator.move_object_with_marker(&temp_co);
    } else
      logger.log(Error, "Object format error: " + temp_co.name);
  } else {
    ui->pushButton_quit->setEnabled(true);
    obj_relocator.stop();
    set_create_object_lineEdit_readOnly(false);
    ui->pushButton_draw->setEnabled(true);
    logger.log(Info, "Moving object with marker: STOP.");
  }
}

void COMannagerDialog::on_pushButton_draw_toggled(bool checked) {
  ui->pushButton_markerpose->setChecked(false);
  if (checked) {
    ui->pushButton_quit->setEnabled(false);
    logger.log(Info, "Drawing object with marker: START.");
    set_create_object_lineEdit_readOnly(true);
    ui->pushButton_markerpose->setEnabled(false);
    obj_drawer.draw_object(&temp_co);
    //    if (collision_objects_mannager.isvalid(temp_co)) {
    //      obj_drawer.draw_object(&temp_co);
    //    } else
    //      logger.log(Error, "Object format error: " + temp_co.name);
  } else {
    ui->pushButton_quit->setEnabled(true);
    obj_drawer.stop();
    set_create_object_lineEdit_readOnly(false);
    ui->pushButton_markerpose->setEnabled(true);
    logger.log(Info, "Drawing object with marker: STOP.");
  }
}

void COMannagerDialog::updatePosition(vector<double> pose) {
  std::stringstream ss;
  for (double n : pose) ss << n << ",";
  string pose_str = ss.str();
  pose_str.pop_back();
  ui->lineEdit_pose->setText(QString(pose_str.c_str()));
  ss.str("");
}

void COMannagerDialog::updateDimension(vector<double> dims) {
  std::stringstream ss;
  for (double n : dims) ss << n << ",";
  string dims_str = ss.str();
  dims_str.pop_back();
  ui->lineEdit_dimensions->setText(QString(dims_str.c_str()));
  ss.str("");
}

void COMannagerDialog::on_pushButton_quit_clicked() {
  obj_relocator.stop();
  obj_drawer.stop();
}

void COMannagerDialog::on_tabWidget_currentChanged(int index) {
  if (index == 1) {
    obj_relocator.stop();
    ui->comboBox_name->clear();
    ui->comboBox_name->addItem(QString(""));
    vector<string> names = collision_objects_mannager.get_objects_names();
    for (string name : names) ui->comboBox_name->addItem(QString(name.c_str()));
  }
}

void COMannagerDialog::on_lineEdit_pose2_editingFinished() {
  string selected_obj = ui->comboBox_name->currentText().toStdString();
  vector<double> new_pose = string2vector<double>(
      "[" + ui->lineEdit_pose2->text().toStdString() + "]");
  collision_objects_mannager.modify_object_pose(selected_obj, new_pose);
  collision_objects_mannager.update();
}

void COMannagerDialog::on_pushButton_markerPose2_toggled(bool checked) {
  if (checked) {
    ui->pushButton_quit->setEnabled(false);
    string selected_obj = ui->comboBox_name->currentText().toStdString();
    collision_object *obj =
        collision_objects_mannager.get_named_object(selected_obj);
    logger.log(Info, "Moving object with marker: START.");
    ui->lineEdit_pose2->setReadOnly(true);
    ui->comboBox_name->setEditable(false);
    ui->tab_createnewobject->setEnabled(false);
    if (collision_objects_mannager.isvalid(*obj)) {
      obj_relocator.move_object_with_marker(obj);
    } else
      logger.log(Error, "Object format error: " + obj->name);
  } else {
    ui->pushButton_quit->setEnabled(true);
    obj_relocator.stop();
    ui->lineEdit_pose2->setReadOnly(false);
    ui->comboBox_name->setEditable(true);
    ui->tab_createnewobject->setEnabled(true);
    logger.log(Info, "Moving object with marker: STOP.");
  }
}
