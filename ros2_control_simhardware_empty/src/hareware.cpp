#include <limits>
#include <vector>

#include "hardware/hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware
{
    constexpr double POSITION_MIN = 0.0;
    constexpr double POSITION_MAX = 1.57;
    constexpr double POSITION_STEP = 0.01745;


    hardware_interface::CallbackReturn SimHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // TODO(anyone): read parameters and initialize the hardware
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimHardwareInterface::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SimHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            hw_states_[i] = stod(info_.joints[i].state_interfaces[0].initial_value);
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                // TODO(anyone): insert correct interfaces
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i * 2]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SimHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                // TODO(anyone): insert correct interfaces
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn SimHardwareInterface::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to receive commands
        hw_commands_ = hw_states_;
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to stop receiving commands

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SimHardwareInterface::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // TODO(anyone): read robot states
        for (auto i = 0; i < 1; i++)
        {
            hw_commands_[i] = hw_states_[i];
            printf("read:command:%f\n", hw_commands_[i]);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimHardwareInterface::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // TODO(anyone): write robot's commands;
        double position = hw_commands_[0];
        static bool increasing = true;
        printf("write:position: %f\n", position);
        if (increasing == true)
        {
            printf("+\n");
            position += POSITION_STEP;
            if (position >= POSITION_MAX)
            {

                position = POSITION_MAX;
                increasing = false;
            }
        }
        if (increasing == false)
        {
            printf("-\n");
            position -= POSITION_STEP;
            if (position <= POSITION_MIN)
            {
                position = POSITION_MIN;
                increasing = true;
            }
        }
        hw_commands_[0] = position;
        hw_states_[0] = position;

        for (auto i = 0; i < 1; i++)
        {
            printf("write: states:%f\n", hw_states_[i]);
        }
        return hardware_interface::return_type::OK;
    }

}  // namespace hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    hardware::SimHardwareInterface, hardware_interface::SystemInterface)