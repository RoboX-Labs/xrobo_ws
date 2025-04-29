#include "robocon_desktop/mainwindow.h"
#include "../ui/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
    : QMainWindow(parent), ui(new Ui::MainWindow), rviz_ros_node_(rviz_ros_node)
{
    ui->setupUi(this);

    cmdvel_pub_ = rviz_ros_node_.lock()->get_raw_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    setupShortcuts();
    setupRvizWidget();
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::setupShortcuts()
{
    QShortcut* wShortcut = new QShortcut(QKeySequence("W"), this);
    connect(wShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 1.0;
        twist_msg->linear.y = 0.0;
        twist_msg->angular.z = 0.0;
        cmdvel_pub_->publish(*twist_msg);
    });

    QShortcut* sShortcut = new QShortcut(QKeySequence("S"), this);
    connect(sShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = -1.0;
        twist_msg->linear.y = 0.0;
        twist_msg->angular.z = 0.0;
        cmdvel_pub_->publish(*twist_msg);
    });

    QShortcut* aShortcut = new QShortcut(QKeySequence("A"), this);
    connect(aShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->linear.y = 1.0;
        twist_msg->angular.z = 0.0;
        cmdvel_pub_->publish(*twist_msg);
    });

    QShortcut* dShortcut = new QShortcut(QKeySequence("D"), this);
    connect(dShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->linear.y = -1.0;
        twist_msg->angular.z = 0.0;
        cmdvel_pub_->publish(*twist_msg);
    });

    QShortcut* jShortcut = new QShortcut(QKeySequence("J"), this);
    connect(jShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->linear.y = 0.0;
        twist_msg->angular.z = 1.0;
        cmdvel_pub_->publish(*twist_msg);
    });

    QShortcut* lShortcut = new QShortcut(QKeySequence("L"), this);
    connect(lShortcut, &QShortcut::activated, this, [this]() {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 0.0;
        twist_msg->linear.y = 0.0;
        twist_msg->angular.z = -1.0;
        cmdvel_pub_->publish(*twist_msg);
    });
}

void MainWindow::setupRvizWidget()
{
    ui->rvizwidget->
}

void MainWindow::on_pushButton_clicked()
{
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 1.0;
    twist_msg->angular.z = 0.0;
    cmdvel_pub_->publish(*twist_msg); 
}
