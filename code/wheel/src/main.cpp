#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/utilities.hpp>



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // 唯一初始化ROS 2
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    int result = a.exec();
    rclcpp::shutdown();  // 清理ROS资源
    return result;
}