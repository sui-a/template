#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <string>
#include <vector>

//存储两个输入
std::vector<std::string> wheel_position;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);  // 必须首先调用，初始化UI控件
    
    // 创建ROS节点（不重复调用rclcpp::init）
    node_ = rclcpp::Node::make_shared("qt_ros2_input_demo");
    setupROS();
    initSpin();  // 启动ROS事件循环
}

MainWindow::~MainWindow()
{
    spin_timer_.stop();
    delete ui;
}

void MainWindow::setupROS()
{
    // 创建发布者（发布到"input_topic"）
    publisher_ = node_->create_publisher<std_msgs::msg::String>("wheel_joint", 10);
}

void MainWindow::initSpin()
{
    // 定时器触发ROS事件处理（非阻塞Qt主线程）
    spin_timer_.setInterval(10);  // 10ms间隔
    QObject::connect(&spin_timer_, &QTimer::timeout, [this]() {
        rclcpp::spin_some(node_);
    });
    spin_timer_.start();
}

void MainWindow::on_confirmButton_clicked()
{
    // 按钮点击时发布两个输入框的内容
    std_msgs::msg::String msg;
    QString input1 = ui->lineEdit1->text();
    QString input2 = ui->lineEdit2->text();
    msg.data = "Input1: " + input1.toStdString() + ", Input2: " + input2.toStdString();
    
    publisher_->publish(msg);
    qDebug() << "Published:" << msg.data.c_str();
}