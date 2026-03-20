#include "my_bot_hw/my_bot_hardware.hpp"

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_bot_hw
{
hardware_interface::CallbackReturn MyBotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call parent class initialization
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read configuration parameters from URDF <hardware><param> block
  left_wheel_name_ = info_.hardware_parameters.at("left_wheel_name");
  right_wheel_name_ = info_.hardware_parameters.at("right_wheel_name");
  serial_device_ = info_.hardware_parameters.at("serial_device");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  timeout_ms_ = std::stoi(info_.hardware_parameters.at("timeout_ms"));
  enc_counts_per_rev_ = std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));

  // Log all parameters for verification
  RCLCPP_INFO(logger_, "left_wheel_name:    %s", left_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "right_wheel_name:   %s", right_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "serial_device:      %s", serial_device_.c_str());
  RCLCPP_INFO(logger_, "baud_rate:          %d", baud_rate_);
  RCLCPP_INFO(logger_, "timeout_ms:         %d", timeout_ms_);
  RCLCPP_INFO(logger_, "enc_counts_per_rev: %d", enc_counts_per_rev_);

  // Initialize state vectors with NaN (indicating no data yet)
  hw_positions_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(logger_, "MyBotHardware initialized with %zu joints", info_.joints.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardware::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "MyBotHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "MyBotHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "MyBotHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "MyBotHardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "MyBotHardware shut down");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyBotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyBotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type MyBotHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyBotHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}
}  // namespace my_bot_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_bot_hw::MyBotHardware,
  hardware_interface::SystemInterface)
