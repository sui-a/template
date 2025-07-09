#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_confirmButton_clicked();  // 确认按钮点击槽函数

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;  // ROS 2节点
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者
    QTimer spin_timer_;  // 定时器处理ROS事件

    void initSpin();  // 初始化ROS事件循环
    void setupROS();  // 设置ROS节点和话题
};

#endif // MAINWINDOW_H