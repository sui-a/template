#ifndef HARDWARE__HARDWARE_HPP_
#define HARDWARE__HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware
{
    class SimHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::vector<double> hw_commands_;
        std::vector<double> hw_states_;
        std::vector<double> joint_states_;

    };

}

#endif