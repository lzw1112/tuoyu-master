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
#include "../include/cyrobot_monitor/main_window.hpp"
#include <QDesktopWidget>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    cyrobot_monitor::MainWindow w(argc,argv);
    QDesktopWidget *desktop = QApplication::desktop();
    w.resize(desktop->screen(0)->width(), desktop->screen(0)->height());
    w.move((desktop->screen(0)->width() - w.width())/2,
            (desktop->screen(0)->height() - w.height())/2);
w.setWindowFlags(Qt::WindowStaysOnBottomHint);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
