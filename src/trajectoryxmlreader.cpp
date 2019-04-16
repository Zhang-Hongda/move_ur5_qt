#include <QFile>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include "../include/move_ur5_qt/trajectoryxmlreader.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

trajectoryXMLReader::trajectoryXMLReader(QWidget *parent)
    : QMainWindow(parent) {
  setWindowTitle(tr("XML Reader"));

  treeWidget = new QTreeWidget(this);
  QStringList headers;
  headers << "pointIndex"
          << "x"
          << "y"
          << "z"
          << "qw"
          << "qx"
          << "qy"
          << "qz";
  treeWidget->setHeaderLabels(headers);
  setCentralWidget(treeWidget);
}

trajectoryXMLReader::~trajectoryXMLReader() {}

bool trajectoryXMLReader::readFile(
    const QString &fileName, std::vector<geometry_msgs::Pose> &waypoints) {
  QFile file(fileName);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    QMessageBox::critical(this, tr("Error"),
                          tr("Cannot read file %1").arg(fileName));
    return false;
  }
  reader.setDevice(&file);
  while (!reader.atEnd()) {
    if (reader.isStartElement()) {
      if (reader.name() == "trajectory") {
        readTrajectoryElement(waypoints);
      } else {
        reader.raiseError(tr("Not a valid trajectory file"));
      }
    } else {
      reader.readNext();
    }
  }
  file.close();
  if (reader.hasError()) {
    QMessageBox::critical(this, tr("Error"),
                          tr("Failed to parse file %1").arg(fileName));
    return false;
  } else if (file.error() != QFile::NoError) {
    QMessageBox::critical(this, tr("Error"),
                          tr("Cannot read file %1").arg(fileName));
    return false;
  }
  return true;
}

bool trajectoryXMLReader::readFile(
    std::vector<geometry_msgs::Pose> &waypoints) {
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), ".",
                                                  tr("XML Files(*.xml)"));
  if (!fileName.isEmpty()) {
    QFile file(fileName);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
      QMessageBox::critical(this, tr("Error"),
                            tr("Cannot read file %1").arg(fileName));
      return false;
    }
    reader.setDevice(&file);
    while (!reader.atEnd()) {
      if (reader.isStartElement()) {
        if (reader.name() == "trajectory") {
          readTrajectoryElement(waypoints);
        } else {
          reader.raiseError(tr("Not a valid trajectory file"));
        }
      } else {
        reader.readNext();
      }
    }
    file.close();
    if (reader.hasError()) {
      QMessageBox::critical(this, tr("Error"),
                            tr("Failed to parse file %1").arg(fileName));
      return false;
    } else if (file.error() != QFile::NoError) {
      QMessageBox::critical(this, tr("Error"),
                            tr("Cannot read file %1").arg(fileName));
      return false;
    }
    return true;
  }
  QMessageBox::critical(this, tr("Error"), tr("Invalid path %1").arg(fileName));
  return false;
}
void trajectoryXMLReader::readTrajectoryElement(
    std::vector<geometry_msgs::Pose> &waypoints) {
  Q_ASSERT(reader.isStartElement() && reader.name() == "trajectory");
  reader.readNext();
  while (!reader.atEnd()) {
    if (reader.isEndElement()) {
      reader.readNext();
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == "point") {
        readPointElement(treeWidget->invisibleRootItem(), waypoints);
      } else {
        skipUnknownElement();
      }
    } else {
      reader.readNext();
    }
  }
}

void trajectoryXMLReader::readPointElement(
    QTreeWidgetItem *parent, std::vector<geometry_msgs::Pose> &waypoints) {
  QTreeWidgetItem *item = new QTreeWidgetItem(parent);
  item->setText(0, QString::number(parent->childCount()));
  reader.readNext();
  geometry_msgs::Pose temp_pose;
  while (!reader.atEnd()) {
    if (reader.isEndElement()) {
      reader.readNext();
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == "position") {
        readPositionElement(item, temp_pose);
      } else if (reader.name() == "orientation") {
        readOrientationElement(item, temp_pose);
      } else {
        skipUnknownElement();
      }
    } else {
      reader.readNext();
    }
  }
  waypoints.push_back(temp_pose);
}

void trajectoryXMLReader::readPositionElement(QTreeWidgetItem *item,
                                              geometry_msgs::Pose &pose) {
  reader.readNext();
  while (!reader.atEnd()) {
    if (reader.isEndElement()) {
      reader.readNext();
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == "x") {
        QString x = reader.readElementText();
        item->setText(1, x);
        pose.position.x = x.toDouble();
        reader.readNext();
      } else if (reader.name() == "y") {
        QString y = reader.readElementText();
        item->setText(2, y);
        pose.position.y = y.toDouble();
        reader.readNext();
      } else if (reader.name() == "z") {
        QString z = reader.readElementText();
        item->setText(3, z);
        pose.position.z = z.toDouble();
        reader.readNext();
      } else {
        skipUnknownElement();
      }
    } else {
      reader.readNext();
    }
  }
}

void trajectoryXMLReader::readOrientationElement(QTreeWidgetItem *item,
                                                 geometry_msgs::Pose &pose) {
  reader.readNext();
  while (!reader.atEnd()) {
    if (reader.isEndElement()) {
      reader.readNext();
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == "qw") {
        QString qw = reader.readElementText();
        item->setText(4, qw);
        pose.orientation.w = qw.toDouble();
        reader.readNext();
      } else if (reader.name() == "qx") {
        QString qx = reader.readElementText();
        item->setText(5, qx);
        pose.orientation.x = qx.toDouble();
        reader.readNext();
      } else if (reader.name() == "qy") {
        QString qy = reader.readElementText();
        pose.orientation.y = qy.toDouble();
        item->setText(6, qy);
        reader.readNext();
      } else if (reader.name() == "qz") {
        QString qz = reader.readElementText();
        item->setText(7, qz);
        pose.orientation.z = qz.toDouble();
        reader.readNext();
      } else {
        skipUnknownElement();
      }
    } else {
      reader.readNext();
    }
  }
}

void trajectoryXMLReader::skipUnknownElement() {
  reader.readNext();
  while (!reader.atEnd()) {
    if (reader.isEndElement()) {
      reader.readNext();
      break;
    }

    if (reader.isStartElement()) {
      skipUnknownElement();
    } else {
      reader.readNext();
    }
  }
}
}
