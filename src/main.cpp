/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>

#include "../include/move_ur5_qt/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  move_ur5_qt::MainWindow w(argc, argv);
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  w.show();
  int result = app.exec();

  return result;
}
