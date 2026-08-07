// Copyright 2024 my_bot_hw
// SPDX-License-Identifier: Apache-2.0

#include "my_bot_hw/stm32_serial_hw.hpp"
#include "my_bot_hw/serial_utils.hpp"

#include <errno.h>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Encoder counts per full wheel revolution (2000 CPR × 34 gear ratio)
static constexpr double COUNTS_PER_REV = 68000.0;
static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr size_t MISSED_FEEDBACK_WARN_CYCLES = 5;
static constexpr std::array<const char *, 10> IMU_INTERFACE_NAMES = {
  "orientation.x",
  "orientation.y",
  "orientation.z",
  "orientation.w",
  "angular_velocity.x",
  "angular_velocity.y",
  "angular_velocity.z",
  "linear_acceleration.x",
  "linear_acceleration.y",
  "linear_acceleration.z"
};

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

  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  }
  if (info_.hardware_parameters.count("feedback_timeout_sec")) {
    feedback_timeout_sec_ = std::stod(info_.hardware_parameters.at("feedback_timeout_sec"));
  }

  if (feedback_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(logger_, "feedback_timeout_sec must be > 0, got %.3f", feedback_timeout_sec_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "serial_port=%s  feedback_timeout=%.3fs",
    serial_port_.c_str(), feedback_timeout_sec_);

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(logger_, "Expected 1 sensor, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name == "left_wheel_joint") {
      left_idx_ = i;
    } else if (info_.joints[i].name == "right_wheel_joint") {
      right_idx_ = i;
    }
  }
  imu_sensor_idx_ = 0;

  const auto & imu_sensor = info_.sensors[imu_sensor_idx_];
  if (imu_sensor.name != "imu_sensor") {
    RCLCPP_ERROR(logger_, "Expected sensor name 'imu_sensor', got '%s'", imu_sensor.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (imu_sensor.state_interfaces.size() != IMU_INTERFACE_NAMES.size()) {
    RCLCPP_ERROR(
      logger_, "imu_sensor must export %zu state interfaces, got %zu", IMU_INTERFACE_NAMES.size(),
      imu_sensor.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < IMU_INTERFACE_NAMES.size(); ++i) {
    if (imu_sensor.state_interfaces[i].name != IMU_INTERFACE_NAMES[i]) {
      RCLCPP_ERROR(
        logger_, "imu_sensor interface %zu mismatch: expected '%s', got '%s'", i,
        IMU_INTERFACE_NAMES[i], imu_sensor.state_interfaces[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);
  imu_states_[0] = 0.0;
  imu_states_[1] = 0.0;
  imu_states_[2] = 0.0;
  imu_states_[3] = 1.0;
  imu_states_[4] = 0.0;
  imu_states_[5] = 0.0;
  imu_states_[6] = 0.0;
  imu_states_[7] = 0.0;
  imu_states_[8] = 0.0;
  imu_states_[9] = 0.0;
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

  // Battery publisher.  This node is never added to an executor — publishing
  // works without one — and is only ever touched from the RX thread.
  if (!battery_node_) {
    battery_node_ = std::make_shared<rclcpp::Node>("my_bot_hw_battery");
    battery_pub_  = battery_node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_raw", rclcpp::QoS(1));
    RCLCPP_INFO(logger_, "Publishing battery on %s",
                battery_pub_->get_topic_name());
  }
  proto_ver_mismatch_logged_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);
  missed_feedback_cycles_ = 0;
  imu_states_[0] = 0.0;
  imu_states_[1] = 0.0;
  imu_states_[2] = 0.0;
  imu_states_[3] = 1.0;
  imu_states_[4] = 0.0;
  imu_states_[5] = 0.0;
  imu_states_[6] = 0.0;
  imu_states_[7] = 0.0;
  imu_states_[8] = 0.0;
  imu_states_[9] = 0.0;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    shared_state_ = SharedState{};
    shared_state_.last_feedback_time = std::chrono::steady_clock::now();
  }

  // Start background RX thread
  rx_running_ = true;
  rx_thread_  = std::thread(&Stm32SerialHardware::rx_thread_fn, this);

  RCLCPP_INFO(logger_, "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_rx_thread();

  // Send zero velocity command to stop the robot
  auto frame = build_vel_cmd(0.0f, 0.0f);
  serial_write(frame.data(), frame.size());

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
  stop_rx_thread();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────── Interface exports ───────────────────────────

std::vector<hardware_interface::StateInterface>
Stm32SerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(4 + IMU_INTERFACE_NAMES.size());
  interfaces.emplace_back(
    info_.joints[left_idx_].name,  hardware_interface::HW_IF_POSITION, &hw_states_[0]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_POSITION, &hw_states_[1]);
  interfaces.emplace_back(
    info_.joints[left_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_states_[2]);
  interfaces.emplace_back(
    info_.joints[right_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[3]);

  const auto & imu_sensor = info_.sensors[imu_sensor_idx_];
  for (const auto & interface : imu_sensor.state_interfaces) {
    double * value_ptr = get_imu_state_ptr(interface.name);
    if (value_ptr == nullptr) {
      throw std::runtime_error("Unsupported imu_sensor state interface: " + interface.name);
    }
    interfaces.emplace_back(imu_sensor.name, interface.name, value_ptr);
  }
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

// ─────────────────────────── Control loop ───────────────────────────

hardware_interface::return_type Stm32SerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Atomically grab and reset the accumulated state from the RX thread
  int32_t ldelta, rdelta;
  int32_t accel_mms2[3];
  int32_t gyro_urad_s[3];
  int32_t yaw_mdeg;
  bool has_fresh_feedback;
  bool has_feedback_ever;
  std::chrono::steady_clock::time_point last_feedback_time;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ldelta       = shared_state_.left_delta_acc;
    rdelta       = shared_state_.right_delta_acc;
    std::memcpy(accel_mms2, shared_state_.accel_mms2, sizeof(accel_mms2));
    std::memcpy(gyro_urad_s, shared_state_.gyro_urad_s, sizeof(gyro_urad_s));
    yaw_mdeg = shared_state_.yaw_mdeg;
    has_fresh_feedback = shared_state_.has_fresh_feedback;
    has_feedback_ever = shared_state_.has_feedback_ever;
    last_feedback_time = shared_state_.last_feedback_time;
    shared_state_.left_delta_acc  = 0;
    shared_state_.right_delta_acc = 0;
    shared_state_.has_fresh_feedback = false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double feedback_age_sec =
    std::chrono::duration<double>(now - last_feedback_time).count();

  if (!has_feedback_ever) {
    if (feedback_age_sec > feedback_timeout_sec_) {
      RCLCPP_ERROR(
        logger_, "No feedback frame received from STM32 within %.3f s after activation",
        feedback_timeout_sec_);
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 2000,
      "Waiting for first feedback frame from STM32");
    return hardware_interface::return_type::OK;
  }

  if (feedback_age_sec > feedback_timeout_sec_) {
    RCLCPP_ERROR(
      logger_, "STM32 feedback timed out: last valid frame was %.3f s ago",
      feedback_age_sec);
    return hardware_interface::return_type::ERROR;
  }

  if (has_fresh_feedback) {
    missed_feedback_cycles_ = 0;
  } else {
    ++missed_feedback_cycles_;
    if (missed_feedback_cycles_ >= MISSED_FEEDBACK_WARN_CYCLES) {
      RCLCPP_WARN_THROTTLE(
        logger_, *rclcpp::Clock::make_shared(), 2000,
        "No fresh STM32 feedback for %zu consecutive control cycles; keeping last sensor state",
        missed_feedback_cycles_);
    }
  }

  // Convert encoder counts → wheel angle increment [rad]
  double left_rad  = static_cast<double>(ldelta) / COUNTS_PER_REV * TWO_PI;
  double right_rad = static_cast<double>(rdelta) / COUNTS_PER_REV * TWO_PI;

  // Update cumulative position [rad]
  hw_states_[0] += left_rad;
  hw_states_[1] += right_rad;

  // Compute velocity [rad/s] from angle / elapsed time
  double dt = period.seconds();
  if (dt > 1e-6) {
    hw_states_[2] = left_rad  / dt;
    hw_states_[3] = right_rad / dt;
  }

  if (has_fresh_feedback) {
    // yaw-only quaternion from STM32 Fusion AHRS (roll=0, pitch=0)
    double yaw_rad = static_cast<double>(yaw_mdeg) * 1e-3 * M_PI / 180.0;
    imu_states_[0] = 0.0;
    imu_states_[1] = 0.0;
    imu_states_[2] = std::sin(yaw_rad / 2.0);
    imu_states_[3] = std::cos(yaw_rad / 2.0);
    imu_states_[4] = static_cast<double>(gyro_urad_s[0]) * 1e-6;
    imu_states_[5] = static_cast<double>(gyro_urad_s[1]) * 1e-6;
    imu_states_[6] = static_cast<double>(gyro_urad_s[2]) * 1e-6;
    imu_states_[7] = static_cast<double>(accel_mms2[0]) * 1e-3;
    imu_states_[8] = static_cast<double>(accel_mms2[1]) * 1e-3;
    imu_states_[9] = static_cast<double>(accel_mms2[2]) * 1e-3;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Stm32SerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_[0] is always left, [1] always right — matching export_command_interfaces()
  auto frame = build_vel_cmd(
    static_cast<float>(hw_commands_[0]),
    static_cast<float>(hw_commands_[1]));

  if (!serial_write(frame.data(), frame.size())) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Failed to send velocity command to STM32");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// ─────────────────────────── Background RX thread ───────────────────────────

void Stm32SerialHardware::rx_thread_fn()
{
  // Ring buffer for frame synchronization
  constexpr size_t BUF_SIZE = 64;
  uint8_t buf[BUF_SIZE];
  size_t  head = 0;   // next write position (circular)
  size_t  fill = 0;   // valid bytes in buf (linear view of unprocessed bytes)

  // We use a simple sliding window: keep a linear byte stream, search for header
  static uint8_t stream[BUF_SIZE * 2];
  size_t stream_fill = 0;

  while (rx_running_) {
    // Read available bytes (non-blocking with poll)
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(serial_fd_, &fds);
    struct timeval tv{0, 20000};  // 20 ms timeout → exit check

    int ret = select(serial_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) continue;  // timeout or error

    uint8_t tmp[32];
    ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
    if (n <= 0) continue;

    // Append to stream buffer, drop oldest bytes if full
    size_t space = sizeof(stream) - stream_fill;
    size_t copy_n = static_cast<size_t>(n);
    if (copy_n > space) {
      // Shift out old data to make room
      size_t shift = copy_n - space;
      memmove(stream, stream + shift, stream_fill - shift);
      stream_fill -= shift;
    }
    memcpy(stream + stream_fill, tmp, copy_n);
    stream_fill += copy_n;

    // Scan the stream for frames.  Multiple frame types share this link, so
    // the scanner must not assume a single frame length: it identifies the
    // type first, then decides whether the frame is complete.
    //
    // The critical rule is the `break` below.  If a frame type is recognised
    // but its bytes have not all arrived, we must wait for more data — not
    // advance one byte.  Advancing would consume the header of a 0x02 that
    // is still being received and silently drop that odometry sample.
    // Only a non-header, an unknown type, a mismatched length field, or a
    // failed checksum justifies stepping forward a byte to resync.
    size_t i = 0;
    while (i + COMM_MIN_FRAME_PREFIX <= stream_fill) {
      if (stream[i] != COMM_HEADER_1 || stream[i+1] != COMM_HEADER_2) {
        i++;
        continue;
      }

      const uint8_t type = stream[i+2];
      const size_t payload_len = frame_payload_len(type);
      if (payload_len == 0u || stream[i+3] != payload_len) {
        i++;   // unknown type or length field does not match — not a frame here
        continue;
      }

      const size_t frame_size = COMM_MIN_FRAME_PREFIX + payload_len;
      if (i + frame_size > stream_fill) {
        break;   // recognised, but not fully received yet — wait, do NOT i++
      }

      if (type == COMM_TYPE_FEEDBACK) {
        FeedbackFrame fb = parse_feedback(stream + i, stream_fill - i);
        if (!fb.valid) { i++; continue; }

        std::lock_guard<std::mutex> lock(state_mutex_);
        shared_state_.left_delta_acc  += fb.left_delta;
        shared_state_.right_delta_acc += fb.right_delta;
        std::memcpy(shared_state_.accel_mms2, fb.accel_mms2, sizeof(shared_state_.accel_mms2));
        std::memcpy(shared_state_.gyro_urad_s, fb.gyro_urad_s, sizeof(shared_state_.gyro_urad_s));
        shared_state_.yaw_mdeg = fb.yaw_mdeg;
        shared_state_.has_fresh_feedback = true;
        shared_state_.has_feedback_ever = true;
        shared_state_.last_feedback_time = std::chrono::steady_clock::now();
      } else {
        BatteryFrame bat = parse_battery(stream + i, stream_fill - i);
        if (!bat.valid) { i++; continue; }
        publish_battery(bat);
      }

      i += frame_size;
    }

    // Consume processed bytes
    if (i > 0) {
      memmove(stream, stream + i, stream_fill - i);
      stream_fill -= i;
    }

    (void)head; (void)fill; (void)buf;  // unused local vars suppressed
  }
}

void Stm32SerialHardware::publish_battery(const BatteryFrame & frame)
{
  if (!battery_pub_) return;

  // proto_ver guards against "edited the ROS side, forgot to reflash".
  // A mismatch must never be accepted silently — the voltage is still put on
  // the wire for diagnosis, but present=false tells consumers not to use it.
  const bool ver_ok = (frame.proto_ver == COMM_PROTO_VER);
  if (!ver_ok) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 5000,
      "STM32 battery frame has proto_ver=%u, this build expects %u. "
      "Reading rejected (present=false) — reflash the firmware.",
      frame.proto_ver, COMM_PROTO_VER);
    proto_ver_mismatch_logged_ = true;
  } else if (proto_ver_mismatch_logged_) {
    RCLCPP_INFO(logger_, "STM32 battery proto_ver now matches (%u)", frame.proto_ver);
    proto_ver_mismatch_logged_ = false;
  }

  if (ver_ok && !frame.vrefint_ok()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 30000,
      "STM32 battery: vrefint_ok=0, VDDA fell back to nominal 3.3 V — "
      "voltage may be off by a few percent.");
  }

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = battery_node_->now();
  msg.voltage = static_cast<float>(frame.voltage_mv) / 1000.0f;
  msg.present = ver_ok && frame.voltage_valid();
  msg.power_supply_status     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  // BatteryState says unknown quantities are NaN.  Leaving them at 0 would
  // read as "empty battery / no current", which is worse than saying nothing.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  msg.temperature     = nan;
  msg.current         = nan;   // current_ma is reserved and always 0 in v1
  msg.charge          = nan;
  msg.capacity        = nan;
  msg.design_capacity = nan;
  msg.percentage      = nan;   // voltage→percent conversion is the next spec

  battery_pub_->publish(msg);
}

void Stm32SerialHardware::stop_rx_thread()
{
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
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

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

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

bool Stm32SerialHardware::serial_write(const uint8_t * data, size_t len)
{
  if (serial_fd_ < 0) return false;
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::write(serial_fd_, data + sent, len - sent);
    if (n > 0) {
      sent += static_cast<size_t>(n);
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // TX buffer full — wait briefly and retry
      struct timeval tv{0, 1000};  // 1 ms
      select(0, nullptr, nullptr, nullptr, &tv);
    } else {
      return false;  // real error
    }
  }
  return true;
}

double * Stm32SerialHardware::get_imu_state_ptr(const std::string & interface_name)
{
  for (size_t i = 0; i < IMU_INTERFACE_NAMES.size(); ++i) {
    if (interface_name == IMU_INTERFACE_NAMES[i]) {
      return &imu_states_[i];
    }
  }
  return nullptr;
}

}  // namespace my_bot_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_bot_hw::Stm32SerialHardware, hardware_interface::SystemInterface)
