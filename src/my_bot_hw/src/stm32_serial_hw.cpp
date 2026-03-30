// Copyright 2024 my_bot_hw
// SPDX-License-Identifier: Apache-2.0

#include "my_bot_hw/stm32_serial_hw.hpp"
#include "my_bot_hw/serial_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace my_bot_hw
{

// ═══════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════

hardware_interface::CallbackReturn Stm32SerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <hardware> <param> tags
  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }
  if (info_.hardware_parameters.count("wheel_separation")) {
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
  }
  if (info_.hardware_parameters.count("wheel_radius")) {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  }
  // Optional: override IMU scale factors from URDF if needed
  if (info_.hardware_parameters.count("imu_gyro_scale")) {
    imu_gyro_scale_ = std::stod(info_.hardware_parameters.at("imu_gyro_scale"));
  }
  if (info_.hardware_parameters.count("imu_accel_scale")) {
    imu_accel_scale_ = std::stod(info_.hardware_parameters.at("imu_accel_scale"));
  }
  if (info_.hardware_parameters.count("imu_quat_scale")) {
    imu_quat_scale_ = std::stod(info_.hardware_parameters.at("imu_quat_scale"));
  }

  RCLCPP_INFO(logger_,
    "serial_port=%s  wheel_sep=%.4f  wheel_radius=%.4f",
    serial_port_.c_str(), wheel_separation_, wheel_radius_);

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name == "left_wheel_joint") {
      left_idx_ = i;
    } else if (info_.joints[i].name == "right_wheel_joint") {
      right_idx_ = i;
    }
  }

  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial(serial_port_)) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Opened serial port %s at 460800 baud", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);
  last_wheel_ts_ = 0;

  // IMU publisher node
  imu_node_ = std::make_shared<rclcpp::Node>("stm32_imu_publisher");
  imu_pub_  = imu_node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

  // Start background reader thread
  reader_running_.store(true);
  reader_thread_ = std::thread(&Stm32SerialHardware::reader_thread_func, this);

  RCLCPP_INFO(logger_, "Hardware activated — streaming from STM32 at 50 Hz");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop reader thread
  reader_running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }

  // Send zero-RPM command to stop robot
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00};
  serial_write(build_frame(0x01, data));

  imu_pub_.reset();
  imu_node_.reset();

  RCLCPP_INFO(logger_, "Hardware deactivated, robot stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();
  RCLCPP_INFO(logger_, "Serial port closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reader_running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
//  Interface exports
// ═══════════════════════════════════════════════════════════════

std::vector<hardware_interface::StateInterface>
Stm32SerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
    info_.joints[left_idx_].name,  hardware_interface::HW_IF_POSITION, &hw_states_[0]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_POSITION, &hw_states_[1]);
  interfaces.emplace_back(
    info_.joints[left_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_states_[2]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[3]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Stm32SerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    info_.joints[left_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
  return interfaces;
}

// ═══════════════════════════════════════════════════════════════
//  Control loop
// ═══════════════════════════════════════════════════════════════

hardware_interface::return_type Stm32SerialHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // ── Copy latest wheel state ────────────────────────────────────────
  WheelState wheel_copy;
  ImuState   imu_copy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    wheel_copy = latest_wheel_;
    imu_copy   = latest_imu_;
    latest_wheel_.fresh = false;
    latest_imu_.fresh   = false;
  }

  if (wheel_copy.fresh) {
    hw_states_[2] = wheel_copy.left_rads;
    hw_states_[3] = wheel_copy.right_rads;

    // Use hardware timestamp for precise position integration
    if (last_wheel_ts_ != 0) {
      // uint32_t subtraction handles wrapping correctly
      uint32_t dt_us = wheel_copy.timestamp_us - last_wheel_ts_;
      double dt_s    = static_cast<double>(dt_us) * 1e-6;
      if (dt_s > 0.0 && dt_s < 0.5) {  // sanity: dt must be in (0, 500ms)
        hw_states_[0] += hw_states_[2] * dt_s;  // left  position  [rad]
        hw_states_[1] += hw_states_[3] * dt_s;  // right position  [rad]
      }
    }
    last_wheel_ts_ = wheel_copy.timestamp_us;
  }

  // ── Publish /imu (from control thread — safe for ROS2 publisher) ───
  if (imu_copy.fresh && imu_pub_) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = time;
    msg.header.frame_id = "imu_link";

    // Orientation from XKF3 quaternion (q = [w, x, y, z])
    msg.orientation.w = imu_copy.quat[0];
    msg.orientation.x = imu_copy.quat[1];
    msg.orientation.y = imu_copy.quat[2];
    msg.orientation.z = imu_copy.quat[3];
    // Covariance: diagonal, tuned to IMU module accuracy (adjust after testing)
    msg.orientation_covariance = {
      0.01, 0.0,  0.0,
      0.0,  0.01, 0.0,
      0.0,  0.0,  0.05  // yaw slightly less accurate
    };

    // Angular velocity [rad/s]
    msg.angular_velocity.x = imu_copy.gyro[0];
    msg.angular_velocity.y = imu_copy.gyro[1];
    msg.angular_velocity.z = imu_copy.gyro[2];
    msg.angular_velocity_covariance = {
      0.02, 0.0,  0.0,
      0.0,  0.02, 0.0,
      0.0,  0.0,  0.02
    };

    // Linear acceleration [m/s²]
    msg.linear_acceleration.x = imu_copy.acc[0];
    msg.linear_acceleration.y = imu_copy.acc[1];
    msg.linear_acceleration.z = imu_copy.acc[2];
    msg.linear_acceleration_covariance = {
      0.05, 0.0,  0.0,
      0.0,  0.05, 0.0,
      0.0,  0.0,  0.05
    };

    imu_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Stm32SerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_ come from diff_drive_controller in [rad/s]
  double left_rads  = hw_commands_[left_idx_];
  double right_rads = hw_commands_[right_idx_];

  // rad/s → RPM: RPM = ω × 60 / (2π)
  double left_rpm  = left_rads  * 60.0 / (2.0 * M_PI);
  double right_rpm = right_rads * 60.0 / (2.0 * M_PI);

  // Scale by ×10 → int16, clamp to int16 range
  auto to_int16 = [](double v) -> int16_t {
    return static_cast<int16_t>(
      std::clamp(v * 10.0,
        static_cast<double>(INT16_MIN),
        static_cast<double>(INT16_MAX)));
  };

  int16_t l = to_int16(left_rpm);
  int16_t r = to_int16(right_rpm);

  // Pack big-endian into 4-byte data field
  std::vector<uint8_t> data = {
    static_cast<uint8_t>((l >> 8) & 0xFF),
    static_cast<uint8_t>( l       & 0xFF),
    static_cast<uint8_t>((r >> 8) & 0xFF),
    static_cast<uint8_t>( r       & 0xFF),
  };

  auto frame = build_frame(0x01, data);
  if (!serial_write(frame)) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Failed to send RPM command to STM32");
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════
//  Background reader thread
// ═══════════════════════════════════════════════════════════════

void Stm32SerialHardware::reader_thread_func()
{
  // Frame parser state machine
  enum ParseState { WAIT_HEADER, READ_LEN, READ_BODY };
  ParseState state = WAIT_HEADER;

  uint8_t  frame_buf[64];
  uint8_t  frame_idx      = 0;
  uint8_t  expected_len   = 0;

  while (reader_running_.load()) {
    uint8_t byte;
    if (!read_byte(byte, 100)) {
      continue;  // timeout — loop back to check reader_running_
    }

    switch (state) {
      case WAIT_HEADER:
        if (byte == 0x5A) {
          frame_buf[0] = byte;
          frame_idx    = 1;
          state        = READ_LEN;
        }
        break;

      case READ_LEN:
        frame_buf[frame_idx++] = byte;
        expected_len = byte;
        // Sanity check: valid frame lengths in this protocol are 14 (0x04) and 36 (0x06)
        if (expected_len == 14 || expected_len == 36) {
          state = READ_BODY;
        } else {
          // Unknown length — reset
          state = WAIT_HEADER;
        }
        break;

      case READ_BODY:
        frame_buf[frame_idx++] = byte;
        if (frame_idx >= expected_len) {
          // Full frame received — validate and dispatch
          std::vector<uint8_t> raw(frame_buf, frame_buf + expected_len);
          uint8_t crc_expected = crc8_maxim(raw.data(), expected_len - 1);

          if (raw[expected_len - 1] == crc_expected && raw[2] == 0x01) {
            uint8_t func = raw[3];

            if (func == 0x04 && expected_len == 14) {
              WheelRpmFrame f = parse_wheel_rpm_frame(raw);
              if (f.valid) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_wheel_.timestamp_us = f.timestamp_us;
                // RPM → rad/s: ω = RPM × 2π / 60
                latest_wheel_.left_rads  = f.left_rpm  * (2.0 * M_PI / 60.0);
                latest_wheel_.right_rads = f.right_rpm * (2.0 * M_PI / 60.0);
                latest_wheel_.fresh      = true;
              }
            } else if (func == 0x06 && expected_len == 36) {
              ImuRawFrame f = parse_imu_raw_frame(raw);
              if (f.valid) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_imu_.timestamp_us = f.timestamp_us;

                // Apply scale factors (verify against IMU module datasheet)
                for (int i = 0; i < 3; i++) {
                  latest_imu_.acc[i]  = f.acc[i]  * imu_accel_scale_;
                  latest_imu_.gyro[i] = f.gyro[i] * imu_gyro_scale_;
                }
                latest_imu_.roll_deg  = f.roll  * imu_angle_scale_;
                latest_imu_.pitch_deg = f.pitch * imu_angle_scale_;
                latest_imu_.yaw_deg   = f.yaw   * imu_angle_scale_;

                // Quaternion [W, X, Y, Z]
                latest_imu_.quat[0] = f.quat[0] * imu_quat_scale_;
                latest_imu_.quat[1] = f.quat[1] * imu_quat_scale_;
                latest_imu_.quat[2] = f.quat[2] * imu_quat_scale_;
                latest_imu_.quat[3] = f.quat[3] * imu_quat_scale_;
                latest_imu_.fresh   = true;
              }
            }
          }
          // Reset for next frame
          state = WAIT_HEADER;
        }
        break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  Serial helpers
// ═══════════════════════════════════════════════════════════════

bool Stm32SerialHardware::open_serial(const std::string & port)
{
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(logger_, "open(%s) failed: %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 460800 8N1 raw mode
  cfmakeraw(&tty);
  cfsetispeed(&tty, B460800);
  cfsetospeed(&tty, B460800);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;  // non-blocking; select() used for timeouts

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void Stm32SerialHardware::close_serial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool Stm32SerialHardware::serial_write(const std::vector<uint8_t> & data)
{
  if (serial_fd_ < 0) { return false; }
  ssize_t written = ::write(serial_fd_, data.data(), data.size());
  return written == static_cast<ssize_t>(data.size());
}

bool Stm32SerialHardware::read_byte(uint8_t & byte, int timeout_ms)
{
  if (serial_fd_ < 0) { return false; }

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(serial_fd_, &fds);

  struct timeval tv{};
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(serial_fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (ret <= 0) { return false; }

  ssize_t n = ::read(serial_fd_, &byte, 1);
  return n == 1;
}

}  // namespace my_bot_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_bot_hw::Stm32SerialHardware, hardware_interface::SystemInterface)
