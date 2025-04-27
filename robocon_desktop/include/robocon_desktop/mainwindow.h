#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node = {});
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};
#endif // MAINWINDOW_H
