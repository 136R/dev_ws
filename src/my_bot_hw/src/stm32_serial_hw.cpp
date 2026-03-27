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

// ─────────────────────────── Lifecycle ───────────────────────────

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

  RCLCPP_INFO(logger_, "serial_port=%s  wheel_sep=%.4f  wheel_radius=%.4f",
    serial_port_.c_str(), wheel_separation_, wheel_radius_);

  // Resolve joint indices by name (defensive: do not assume ordering)
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
  RCLCPP_INFO(logger_, "Opened serial port: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);

  // Create a standalone ROS node for IMU publishing (hardware interface has no built-in publisher API)
  imu_node_ = std::make_shared<rclcpp::Node>("stm32_imu_publisher");
  imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

  RCLCPP_INFO(logger_, "Hardware activated, IMU publisher ready on /imu");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero velocity to stop the robot
  auto stop_frame = build_frame(0x01, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
  serial_write(stop_frame);

  // Clean up IMU publisher
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
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────── Interface exports ───────────────────────────

std::vector<hardware_interface::StateInterface>
Stm32SerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  // hw_states_[0] = left position, [1] = right position
  // hw_states_[2] = left velocity, [3] = right velocity
  interfaces.emplace_back(
    info_.joints[left_idx_].name, hardware_interface::HW_IF_POSITION, &hw_states_[0]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_POSITION, &hw_states_[1]);
  interfaces.emplace_back(
    info_.joints[left_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[2]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[3]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Stm32SerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  // hw_commands_[0] = left wheel velocity cmd [rad/s]
  // hw_commands_[1] = right wheel velocity cmd [rad/s]
  interfaces.emplace_back(
    info_.joints[left_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
  return interfaces;
}

// ─────────────────────────── Control loop ───────────────────────────

hardware_interface::return_type Stm32SerialHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Send odometry query (0x09)
  auto query = build_frame(0x09, {});
  if (!serial_write(query)) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Failed to send odometry query to STM32");
    return hardware_interface::return_type::OK;  // non-fatal: keep last state
  }

  // Read 0x0A response with 18ms timeout (leaves margin in 20ms cycle)
  auto raw = read_odom_frame(18);
  if (raw.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Odometry response timeout from STM32");
    return hardware_interface::return_type::OK;
  }

  OdomFrame odom = parse_odom_frame(raw);
  if (!odom.valid) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Invalid CRC in STM32 odometry frame");
    return hardware_interface::return_type::OK;
  }

  // Decompose chassis vx/vyaw → left/right wheel velocities [m/s]
  // For differential drive:
  //   v_left  = vx - vyaw * (wheel_sep / 2)
  //   v_right = vx + vyaw * (wheel_sep / 2)
  double half_sep = wheel_separation_ / 2.0;
  double v_left_mps  = odom.vx - odom.vyaw * half_sep;
  double v_right_mps = odom.vx + odom.vyaw * half_sep;

  // Convert m/s → rad/s for joint velocity state
  hw_states_[2] = v_left_mps  / wheel_radius_;  // left  velocity [rad/s]
  hw_states_[3] = v_right_mps / wheel_radius_;  // right velocity [rad/s]

  // Integrate wheel positions [rad]
  double dt = period.seconds();
  hw_states_[0] += hw_states_[2] * dt;  // left  position
  hw_states_[1] += hw_states_[3] * dt;  // right position

  // ── Build /imu message from 0x0A frame (yaw orientation + angular velocity z) ──
  // odom.vyaw is a direct angular velocity measurement [rad/s]
  // odom.yaw_deg is the STM32 accumulated heading [deg]
  if (imu_pub_) {
    double yaw_rad = odom.yaw_deg * M_PI / 180.0;
    // pitch and roll not available from 0x0A; for a ground robot they are ~0
    double cy = std::cos(yaw_rad * 0.5), sy = std::sin(yaw_rad * 0.5);
    // cp = cos(0) = 1, sp = sin(0) = 0, cr = cos(0) = 1, sr = sin(0) = 0
    // Simplifies to pure-yaw quaternion: w=cy, x=0, y=0, z=sy
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = time;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = cy;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sy;
    imu_msg.orientation_covariance = {0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.01};
    imu_msg.angular_velocity.z = odom.vyaw;  // rad/s, direct measurement from STM32
    imu_msg.angular_velocity_covariance = {0.001, 0, 0,  0, 0.001, 0,  0, 0, 0.001};
    // No linear acceleration data from 0x0A — signal EKF to ignore
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Stm32SerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_ are in rad/s from diff_drive_controller
  double v_left_rads  = hw_commands_[0];
  double v_right_rads = hw_commands_[1];

  // Convert rad/s → m/s
  double v_left_mps  = v_left_rads  * wheel_radius_;
  double v_right_mps = v_right_rads * wheel_radius_;

  // Compute chassis linear and angular velocity
  double vx   = (v_left_mps + v_right_mps) / 2.0;
  double vyaw = (v_right_mps - v_left_mps) / wheel_separation_;

  // Scale to int16 (*1000), clamp to int16 range
  auto to_int16 = [](double v) -> int16_t {
    return static_cast<int16_t>(
      std::clamp(v * 1000.0, static_cast<double>(INT16_MIN), static_cast<double>(INT16_MAX)));
  };

  int16_t vx_cmd   = to_int16(vx);
  int16_t vy_cmd   = 0;  // differential drive has no lateral velocity
  int16_t vyaw_cmd = to_int16(vyaw);

  // Pack big-endian (MSB first, per protocol: "Byte1=X MSB, Byte2=X LSB")
  std::vector<uint8_t> data = {
    static_cast<uint8_t>((vx_cmd   >> 8) & 0xFF),
    static_cast<uint8_t>( vx_cmd         & 0xFF),
    static_cast<uint8_t>((vy_cmd   >> 8) & 0xFF),
    static_cast<uint8_t>( vy_cmd         & 0xFF),
    static_cast<uint8_t>((vyaw_cmd >> 8) & 0xFF),
    static_cast<uint8_t>( vyaw_cmd       & 0xFF),
  };

  auto frame = build_frame(0x01, data);
  if (!serial_write(frame)) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Failed to send velocity command to STM32");
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────── Serial helpers ───────────────────────────

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

  // 115200 8N1, raw mode, no flow control
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;  // non-blocking reads; we use select() for timeouts

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);  // flush stale data
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
  if (serial_fd_ < 0) {return false;}
  ssize_t written = ::write(serial_fd_, data.data(), data.size());
  return written == static_cast<ssize_t>(data.size());
}

std::vector<uint8_t> Stm32SerialHardware::serial_read_timeout(
  size_t expected_len, int timeout_ms)
{
  std::vector<uint8_t> buf;
  buf.reserve(expected_len);

  auto deadline_us = static_cast<int64_t>(timeout_ms) * 1000;

  while (buf.size() < expected_len && deadline_us > 0) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(serial_fd_, &fds);

    struct timeval tv{};
    tv.tv_sec  = deadline_us / 1000000;
    tv.tv_usec = deadline_us % 1000000;

    int ret = select(serial_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) {break;}  // timeout or error

    uint8_t byte;
    ssize_t n = ::read(serial_fd_, &byte, 1);
    if (n > 0) {
      buf.push_back(byte);
      deadline_us -= 1000;  // rough accounting; select updates tv on Linux
    }
  }
  return buf;
}

std::vector<uint8_t> Stm32SerialHardware::read_odom_frame(int timeout_ms)
{
  if (serial_fd_ < 0) {return {};}

  // Drain until we find 0x5A start byte, then read 11 more bytes
  auto deadline_ms = timeout_ms;

  while (deadline_ms > 0) {
    auto header = serial_read_timeout(1, std::min(deadline_ms, 5));
    deadline_ms -= 5;
    if (header.empty()) {continue;}
    if (header[0] != 0x5A) {continue;}  // skip garbage

    // Read remaining 11 bytes of the 12-byte frame
    auto rest = serial_read_timeout(11, deadline_ms);
    if (rest.size() != 11) {return {};}

    std::vector<uint8_t> frame;
    frame.reserve(12);
    frame.push_back(0x5A);
    frame.insert(frame.end(), rest.begin(), rest.end());
    return frame;
  }
  return {};
}

}  // namespace my_bot_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_bot_hw::Stm32SerialHardware, hardware_interface::SystemInterface)
