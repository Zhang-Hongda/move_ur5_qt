#ifndef TRAJECTORYXMLWRITER_HPP
#define TRAJECTORYXMLWRITER_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#endif
#include <QMainWindow>
#include <QXmlStreamWriter>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace move_ur5_qt {

/*****************************************************************************
** Class
*****************************************************************************/
class trajectoryXMLWriter : public QMainWindow {
  Q_OBJECT
 public:
  explicit trajectoryXMLWriter();
  ~trajectoryXMLWriter();
  bool writeFile(std::vector<geometry_msgs::Pose> waypoints);
  bool writeFile(std::vector<geometry_msgs::Pose> waypoints,
                 const QString &fileName);
  void writePointIndex(geometry_msgs::Pose pose);

 private:
  QXmlStreamWriter xmlWriter;
};
}
#endif  // TRAJECTORYXMLWRITER_HPP
