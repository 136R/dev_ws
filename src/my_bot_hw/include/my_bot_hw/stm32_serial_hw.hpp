// Copyright 2024 my_bot_hw
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <atomic>
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
  // ── Serial low-level helpers ───────────────────────────────────────
  bool open_serial(const std::string & port);
  void close_serial();
  bool serial_write(const std::vector<uint8_t> & data);

  // ── Background reader thread ───────────────────────────────────────
  // Runs continuously; parses 0x04 / 0x06 broadcast frames from STM32.
  // Updates latest_wheel_ and latest_imu_ under state_mutex_.
  void reader_thread_func();

  // Byte-level helpers used by reader thread
  // Returns true if a byte was read within timeout_ms; false on timeout/error.
  bool read_byte(uint8_t & byte, int timeout_ms);

  // ── Parameters (from URDF <param>) ────────────────────────────────
  std::string serial_port_{"/dev/ttyS7"};
  double wheel_separation_{0.171};
  double wheel_radius_{0.0325};

  // IMU physical scale factors (must match STM32 Config.h)
  double imu_gyro_scale_{0.1 * 0.01745329};    // 0.1 dps/LSB → rad/s
  double imu_accel_scale_{9.80665 / 1000.0};   // 1 mg/LSB → m/s²
  double imu_quat_scale_{1.0 / 32768.0};       // Q15
  double imu_angle_scale_{0.1};                // 0.1 °/LSB

  // ── Hardware state ─────────────────────────────────────────────────
  int serial_fd_{-1};
  size_t left_idx_{0};
  size_t right_idx_{1};
  std::array<double, 4> hw_states_{};    // [0]=left_pos [1]=right_pos [2]=left_vel [3]=right_vel
  std::array<double, 2> hw_commands_{}; // [0]=left_vel_cmd [1]=right_vel_cmd [rad/s]

  // ── Background reader thread ───────────────────────────────────────
  std::thread reader_thread_;
  std::atomic<bool> reader_running_{false};

  // ── Shared state (protected by state_mutex_) ───────────────────────
  std::mutex state_mutex_;

  struct WheelState {
    uint32_t timestamp_us{0};
    double   left_rads{0.0};
    double   right_rads{0.0};
    bool     fresh{false};
  } latest_wheel_;

  struct ImuState {
    uint32_t timestamp_us{0};
    double   acc[3]{0.0, 0.0, 0.0};    // [m/s²]
    double   gyro[3]{0.0, 0.0, 0.0};   // [rad/s]
    double   roll_deg{0.0};
    double   pitch_deg{0.0};
    double   yaw_deg{0.0};
    double   quat[4]{1.0, 0.0, 0.0, 0.0};  // [w, x, y, z]
    bool     fresh{false};
  } latest_imu_;

  uint32_t last_wheel_ts_{0};  // for precise dt integration

  // ── IMU publisher ──────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Node> imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // ── Logger ────────────────────────────────────────────────────────
  rclcpp::Logger logger_{rclcpp::get_logger("Stm32SerialHardware")};
};

}  // namespace my_bot_hw
