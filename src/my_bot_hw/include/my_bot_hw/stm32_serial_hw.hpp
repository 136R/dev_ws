// Copyright 2024 my_bot_hw
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace my_bot_hw
{

class Stm32SerialHardware : public hardware_interface::SystemInterface
{
public:
  // ── Lifecycle ──────────────────────────────────────────────────────
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

  // ── Interface exports ──────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop ───────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Serial helpers ─────────────────────────────────────────────────
  bool open_serial(const std::string & port);
  void close_serial();
  bool serial_write(const std::vector<uint8_t> & data);
  std::vector<uint8_t> serial_read_timeout(size_t expected_len, int timeout_ms);
  std::vector<uint8_t> read_odom_frame(int timeout_ms);

  // ── Parameters (from URDF <param>) ────────────────────────────────
  std::string serial_port_{"/dev/ttyS7"};
  double wheel_separation_{0.171};
  double wheel_radius_{0.0325};

  // ── Hardware state ─────────────────────────────────────────────────
  int serial_fd_{-1};
  size_t left_idx_{0};
  size_t right_idx_{1};
  std::array<double, 4> hw_states_{};   // [0]=left_pos [1]=right_pos [2]=left_vel [3]=right_vel
  std::array<double, 2> hw_commands_{}; // [0]=left_vel_cmd [1]=right_vel_cmd

  // ── IMU publisher (standalone node, derives data from 0x0A frame) ─
  std::shared_ptr<rclcpp::Node> imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // ── Logger ────────────────────────────────────────────────────────
  rclcpp::Logger logger_{rclcpp::get_logger("Stm32SerialHardware")};
};

}  // namespace my_bot_hw
