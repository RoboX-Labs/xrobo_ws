#include "robocon_desktop/mainwindow.h"
#include "../ui/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
    : QMainWindow(parent), ui(new Ui::MainWindow), rviz_ros_node_(rviz_ros_node)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
}
