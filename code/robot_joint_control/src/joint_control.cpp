#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node 
{
public:
    JointPublisher() : Node("six_joint_publisher") 
    {
        // 初始化发布者，话题名为标准关节状态话题
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // 设置6个关节名称（需与URDF中定义的关节名一致）
        joint_names_ = {"left_wheel1_joint", "left_wheel2_joint", "right_wheel1_joint", "right_wheel2_joint", "arm1_joint", "arm2_rotation_joint", "arm2_prismatic_joint"};
        joint_current_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 初始位置全为0 /当前位置
        joint_target_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

        // 定时器（10Hz频率）
        timer_ = this->create_wall_timer(100ms, [this]() { this->publish_joint_states(); });
        
        
        //开始设置订阅者
        //对轮子
        _subscription.push_back(this->create_subscription<std_msgs::msg::String>(
            "wheel_joint", 10, std::bind(&JointPublisher::whell_deal, this, std::placeholders::_1)));
        
        //对机械臂
        _subscription.push_back(this->create_subscription<std_msgs::msg::String>(
            "arm_joint", 10, std::bind(&JointPublisher::arm_deal, this, std::placeholders::_1)));
    }
    
    //轮函数位置信息获取
    void whell_deal(const std_msgs::msg::String & msg) 
    {
        // 打印关节名称和位置
        RCLCPP_INFO(this->get_logger(), "Received whell JointState:");

        //std::cout << "data: '" << msg.data << "'" << std::endl;
        //Input1: 1, Input2: 2

        std::vector<std::string> str_position;
        std::string str_dat = msg.data;

        size_t pos = str_dat.find(",");

        str_position.push_back(str_dat.substr(8, pos - 8));
        str_dat.erase(0, pos + 1);
        str_position.push_back(str_dat.substr(9, str_dat.size()));

        double left_wheel;
        double right_wheel;

        try 
        {
            left_wheel = std::stod(str_position[0]);
            right_wheel = std::stod(str_position[0]);
        }
        catch (const std::invalid_argument& e) 
        {
            std::cerr << "无效的数字格式" << std::endl;
            return;
        } 
        catch (const std::out_of_range& e) 
        {
            std::cerr << "数值超出范围" << std::endl;
            return;
        }

        //std::cout << "1: " << str_position[0] << std::endl;
        //std::cout << "2: " << str_position[1] << std::endl;

        //对于轮子
        this->joint_target_[0] = left_wheel / 0.05;
        this->joint_target_[1] = this->joint_target_[0];

        this->joint_target_[2] = right_wheel / 0.05;
        this->joint_target_[3] = this->joint_target_[2];
    }

    //对机械臂进行处理
    void arm_deal(const std_msgs::msg::String & msg)
    {
        // 打印关节名称和位置
        RCLCPP_INFO(this->get_logger(), "Received arm JointState:");
        
        std::vector<std::string> str_position;
        std::string str_dat = msg.data;

        size_t pos = str_dat.find(",");

        str_position.push_back(str_dat.substr(8, pos - 8));
        str_dat.erase(0, pos + 1);
        str_position.push_back(str_dat.substr(9, str_dat.size()));

        double angle;
        double elongtion;

        try 
        {
            angle = std::stod(str_position[0]);
            elongtion = std::stod(str_position[0]);
        }
        catch (const std::invalid_argument& e) 
        {
            std::cerr << "无效的数字格式" << std::endl;
            return;
        } 
        catch (const std::out_of_range& e) 
        {
            std::cerr << "数值超出范围" << std::endl;
            return;
        }

        //判断是否越界
        if(angle > 180)
        {
            angle = 180;
        }
        else if(angle < 0)
        {
            angle = 0;
        }

        if(elongtion < 0)
        {
            elongtion = 0;
        }
        else if(elongtion > 0.2)
        {
            elongtion = 0.2;
        }

        joint_target_[6] = elongtion;

        //进行角度转弧度
        joint_target_[4] = -(angle * M_PI / 180.0); 
    }

private:
    void publish_joint_states() 
    {
        //左半边
        if(joint_current_[0] > joint_target_[0])
        {
            joint_current_[0] -= 0.01;
            joint_current_[1] -= 0.01;
        }
        else if(joint_current_[0] < joint_target_[0])
        {
            joint_current_[0] += 0.01;
            joint_current_[1] += 0.01;
        }

        //右半边
        if(joint_current_[2] > joint_target_[2])
        {
            joint_current_[2] -= 0.01;
            joint_current_[3] -= 0.01;
        }
        else if(joint_current_[2] < joint_target_[2])
        {
            joint_current_[2] += 0.01;
            joint_current_[3] += 0.01;
        }

        //左
        if(joint_current_[0] >= 6.283)
        {
            joint_current_[0] -= 6.283;
            joint_target_[0] -= 6.283;
            joint_current_[1] = joint_current_[0];
            joint_target_[1] = joint_target_[0];
        }
        else if(joint_current_[0] < 0)
        {
            joint_current_[0] += 6.283;
            joint_target_[0] += 6.283;
            joint_current_[1] = joint_current_[0];
            joint_target_[1] = joint_target_[0];
        }
        
        //右半边
        if(joint_current_[2] >= 6.283)
        {
            joint_current_[2] -= 6.283;
            joint_target_[2] -= 6.283;
            joint_current_[3] = joint_current_[2];
            joint_target_[3] = joint_target_[2];
        }
        else if(joint_current_[2] < 0)
        {
            joint_current_[2] += 6.283;
            joint_target_[2] += 6.283;
            joint_current_[3] = joint_current_[2];
            joint_target_[3] = joint_target_[2];
        }


        //开始判断角度和长度 位置4和5
        if(joint_current_[4] < joint_target_[4])
        {
            joint_current_[4] += 0.01;
        }
        else if(joint_current_[4] > joint_target_[4])
        {
            joint_current_[4] -= 0.01;
        }

        //判断拉伸轴
        if(joint_current_[6] > joint_target_[6])
        {
            joint_current_[6] -= 0.01;
        }
        else if(joint_current_[6] < joint_target_[6])
        {
            joint_current_[6] += 0.01;
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();      // 时间戳
        msg.header.frame_id = "base_link";  // 参考坐标系
        msg.name = joint_names_;
        msg.position = joint_current_;
        
        publisher_->publish(msg);
    }

    //创建发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_current_;
    std::vector<double> joint_target_;  //目标位置

    //创建订阅者，四个轮子为一个总姿态， 机械臂为一个总姿态
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> _subscription;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}