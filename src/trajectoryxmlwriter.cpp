#include <QFile>
#include <QXmlStreamWriter>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include "../include/move_ur5_qt/trajectoryxmlwriter.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/
trajectoryXMLWriter::trajectoryXMLWriter() {}

trajectoryXMLWriter::~trajectoryXMLWriter() {}

bool trajectoryXMLWriter::writeFile(
    std::vector<geometry_msgs::Pose> waypoints) {
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), ".",
                                                  tr("XML Files(*.xml)"));
  if (!fileName.isEmpty()) {
    QFile file(fileName);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {
      qDebug() << "Error: Cannot write file: "
               << qPrintable(file.errorString());
      return false;
    }
    xmlWriter.setDevice(&file);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("trajectory");
    for (std::vector<geometry_msgs::Pose>::const_iterator it =
             waypoints.begin();
         it != waypoints.end(); it++) {
      writePointIndex(*it);
    }
    xmlWriter.writeEndElement();
    xmlWriter.writeEndDocument();
    file.close();
    if (file.error()) {
      qDebug() << "Error: Cannot write file: "
               << qPrintable(file.errorString());
      return false;
    }
    return true;
  }
  QMessageBox::critical(this, tr("Error"), tr("Invalid path %1").arg(fileName));
  return false;
}

bool trajectoryXMLWriter::writeFile(std::vector<geometry_msgs::Pose> waypoints,
                                    const QString &fileName) {
  QFile file(fileName);
  if (!file.open(QFile::WriteOnly | QFile::Text)) {
    qDebug() << "Error: Cannot write file: " << qPrintable(file.errorString());
    return false;
  }

  xmlWriter.setDevice(&file);
  xmlWriter.setAutoFormatting(true);
  xmlWriter.setAutoFormatting(true);
  xmlWriter.writeStartDocument();
  xmlWriter.writeStartElement("trajectory");
  for (std::vector<geometry_msgs::Pose>::const_iterator it = waypoints.begin();
       it != waypoints.end(); it++) {
    writePointIndex(*it);
  }
  xmlWriter.writeEndElement();
  xmlWriter.writeEndDocument();
  file.close();
  if (file.error()) {
    qDebug() << "Error: Cannot write file: " << qPrintable(file.errorString());
    return false;
  }
  return true;
}

void trajectoryXMLWriter::writePointIndex(geometry_msgs::Pose pose) {
  xmlWriter.writeStartElement("point");

  xmlWriter.writeStartElement("position");
  xmlWriter.writeTextElement("x", QString::number(pose.position.x));
  xmlWriter.writeTextElement("y", QString::number(pose.position.y));
  xmlWriter.writeTextElement("z", QString::number(pose.position.z));
  xmlWriter.writeEndElement();

  xmlWriter.writeStartElement("orientation");
  xmlWriter.writeTextElement("qw", QString::number(pose.orientation.w));
  xmlWriter.writeTextElement("qx", QString::number(pose.orientation.x));
  xmlWriter.writeTextElement("qy", QString::number(pose.orientation.y));
  xmlWriter.writeTextElement("qz", QString::number(pose.orientation.z));
  xmlWriter.writeEndElement();

  xmlWriter.writeEndElement();
}
}
