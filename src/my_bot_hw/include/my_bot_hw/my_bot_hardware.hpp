#ifndef MY_BOT_HW__MY_BOT_HARDWARE_HPP_
#define MY_BOT_HW__MY_BOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_bot_hw
{
class MyBotHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("MyBotHardware");

  // Configuration parameters read from URDF <hardware><param> block
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  std::string serial_device_;
  int baud_rate_;
  int timeout_ms_;
  int enc_counts_per_rev_;

  // State and command interfaces storage
  std::vector<double> hw_positions_;   // Current joint positions (rad)
  std::vector<double> hw_velocities_;  // Current joint velocities (rad/s)
  std::vector<double> hw_commands_;    // Joint velocity commands (rad/s)
};
}  // namespace my_bot_hw

#endif  // MY_BOT_HW__MY_BOT_HARDWARE_HPP_
