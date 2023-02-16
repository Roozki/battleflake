/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#include "../include/mainwindow.h"
#include "ui_mainwindow.h"

// QT
#include "QDebug"
#include "QDir"
#include "QLabel"
#include "QMessageBox"
#include "QPixmap"
#include "QProcess"

MainWindow::MainWindow(QWidget* parent)
  :

    QMainWindow(parent),
    ui(new Ui::MainWindow) {
    ui->setupUi(this);
    this->setWindowTitle("Bobo Interface");
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(twist_values()));
    ;
    timer->start(10);

    // ROS
    ros_f = new RosIntegration();
    qDebug() << "Constructor OK";

    // UI

    // Snowbots Logo
    //QPixmap pixmap("./src/snowbots_ui/resources/snowbot2.png");
    //ui->label_5->setPixmap(pixmap);
    //ui->label_5->show();
    //ui->label_5->setScaledContents(true);
    qDebug() << "Current dir:" << QDir::currentPath();
}
MainWindow::~MainWindow() {
    delete ui;
    qDebug() << "Destructor OK";
}

void MainWindow::twist_values() {
    ui->mot1PWR_lcd->display(cmd_msg.drive1PWR);
    ui->mot2PWR_lcd->display(cmd_msg.drive2PWR);
    ui->angular_lcd->display(twist_message_controller.angular.z);
    ui->linear_lcd->display(twist_message_controller.linear.x);
    ros_f->twist_subscriber();
}
