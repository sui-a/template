#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("my_odom")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OdomPublisher::timer_callback, this)
        );
    }
private:
    void timer_callback()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_footprint";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;

        msg.twist.twist.linear.x = 0.0;
        msg.twist.twist.angular.z = 0.0;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}