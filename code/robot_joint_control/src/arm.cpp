#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <thread>

class RobotWheelNode : public rclcpp::Node 
{
public:
    RobotWheelNode() : Node("robot_arm_node") 
    {
        _arm_position = {0.0, 0.0};

        // 初始化发布者，话题名为 "/wheel_joint_states"，队列大小为10
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/arm_joint", 10);

        // 初始化定时器，以10Hz频率发布数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 100ms = 10Hz
            std::bind(&RobotWheelNode::publish_joint_state, this));

        RCLCPP_INFO(this->get_logger(), "Robot Wheel Node 已启动，正在发布关节状态...");

        // 启动输入线程
        input_thread_ = std::thread(&RobotWheelNode::read_user_input, this);
    }

private:

    void read_user_input() 
    {
        while (rclcpp::ok()) 
        {
            std::string left_input, right_input;
            std::cout << "请输入机械臂角度 ";
            std::getline(std::cin, left_input);
            std::cout << "请输入机械臂伸缩量： ";
            std::getline(std::cin, right_input);

            std::lock_guard<std::mutex> lock(position_mutex_);
            _arm_position[0] = std::stod(left_input);
            _arm_position[1] = std::stod(right_input);
        }
    }

    void publish_joint_state() 
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        auto message = sensor_msgs::msg::JointState();
        
        // 设置消息头（时间戳和坐标系）
        message.header.stamp = this->now();
        message.header.frame_id = "base_link";

        message.name = {"arm_angle", "arm_elongtion"};

        // 设置关节末端位置 角度(0, 180)  伸长度数(0, 0.2)
        message.position = {_arm_position[0], _arm_position[1]};

        // 后续不做处理
        message.velocity = {0.0, 0.0};
        message.effort = {0.0, 0.0};

        // 发布消息
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex position_mutex_;
    std::thread input_thread_;
    std::vector<double> _arm_position;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotWheelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}