#include "robocon_desktop/mainwindow.h"

#include <QApplication>

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    rclcpp::init(argc, argv);
    auto ros_node_abs =
        std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");

    MainWindow w(nullptr, ros_node_abs);
    w.show();

    return a.exec();
}
