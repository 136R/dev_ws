// Copyright 2024 my_bot_hw
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  bool serial_write(const uint8_t * data, size_t len);
  double * get_imu_state_ptr(const std::string & interface_name);

  // ── Background RX thread ───────────────────────────────────────────
  void rx_thread_fn();
  void stop_rx_thread();

  // ── Parameters (from URDF <param>) ────────────────────────────────
  std::string serial_port_{"/dev/ttyS7"};
  double feedback_timeout_sec_{0.5};

  // ── Hardware state ─────────────────────────────────────────────────
  int serial_fd_{-1};
  size_t left_idx_{0};
  size_t right_idx_{1};
  size_t imu_sensor_idx_{0};
  std::array<double, 4> hw_states_{};   // [0]=left_pos [1]=right_pos [2]=left_vel [3]=right_vel
  std::array<double, 2> hw_commands_{}; // [0]=left_vel_cmd [1]=right_vel_cmd [rad/s]
  std::array<double, 10> imu_states_{
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0};
  size_t missed_feedback_cycles_{0};

  // ── Shared state between RX thread and read() ─────────────────────
  // Protected by state_mutex_
  struct SharedState {
    int32_t left_delta_acc{0};   // accumulated encoder counts since last read()
    int32_t right_delta_acc{0};
    int32_t accel_mms2[3]{0, 0, 0};
    int32_t gyro_urad_s[3]{0, 0, 0};
    bool    has_fresh_feedback{false};
    bool    has_feedback_ever{false};
    std::chrono::steady_clock::time_point last_feedback_time{};
  };
  std::mutex   state_mutex_;
  SharedState  shared_state_{};

  // ── RX thread lifecycle ────────────────────────────────────────────
  std::thread           rx_thread_;
  std::atomic<bool>     rx_running_{false};
  // ── Logger ────────────────────────────────────────────────────────
  rclcpp::Logger logger_{rclcpp::get_logger("Stm32SerialHardware")};
};

}  // namespace my_bot_hw
