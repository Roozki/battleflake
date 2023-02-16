/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#include "../include/mainwindow.h"
#include "ros/ros.h"
#include <QApplication>
#include <QtGui>

int main(int argc, char** argv) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "bb_gui");

    MainWindow w;
    w.show();

    qDebug() << "main";

    return a.exec();
}
