#ifndef TRAJECTORYXMLREADER_HPP
#define TRAJECTORYXMLREADER_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#endif
#include <QMainWindow>
#include <QXmlStreamReader>
#include <QTreeWidgetItem>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Class
*****************************************************************************/
class trajectoryXMLReader : public QMainWindow {
  Q_OBJECT
 public:
  explicit trajectoryXMLReader(QWidget *parent = 0);
  ~trajectoryXMLReader();
  bool readFile(const QString &fileName,
                std::vector<geometry_msgs::Pose> &waypoints);
  bool readFile(std::vector<geometry_msgs::Pose> &waypoints);

 private:
  void readTrajectoryElement(std::vector<geometry_msgs::Pose> &waypoints);
  void readPointElement(QTreeWidgetItem *parent,
                        std::vector<geometry_msgs::Pose> &waypoints);
  void readPositionElement(QTreeWidgetItem *parent, geometry_msgs::Pose &pose);
  void readOrientationElement(QTreeWidgetItem *parent,
                              geometry_msgs::Pose &pose);
  void skipUnknownElement();

  QTreeWidget *treeWidget;
  QXmlStreamReader reader;
};
}
#endif  // TRAJECTORYXMLREADER_HPP
