#ifndef MY_BOT_HW__MY_BOT_HW_INTERFACE_HPP_
#define MY_BOT_HW__MY_BOT_HW_INTERFACE_HPP_

#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_bot_hw
{
class MyBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyBotHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;
  double wheel_radius_;
  double wheel_separation_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  bool openSerialPort();
  void closeSerialPort();
  bool sendSpeedCommand(double linear_x, double angular_z);
  bool requestOdometry();
  bool receiveOdometry(double &linear_x, double &linear_y, double &angular_z);
  unsigned char calculateCRC8(unsigned char *data, unsigned char len);
};
}

#endif
