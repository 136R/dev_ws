#include "my_bot_hw/my_bot_hw_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace my_bot_hw
{

unsigned char MyBotHardwareInterface::calculateCRC8(unsigned char *data, unsigned char len)
{
  uint8_t crc = 0;
  while(len--) {
    crc ^= *data++;
    for(int i = 0; i < 8; i++) {
      if(crc & 0x01) crc = (crc >> 1) ^ 0x8C;
      else crc >>= 1;
    }
  }
  return crc;
}

bool MyBotHardwareInterface::openSerialPort()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyBotHardwareInterface"), "Failed to open serial port: %s", serial_port_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyBotHardwareInterface"), "Failed to get serial attributes");
    close(serial_fd_);
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~IGNBRK;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  tty.c_lflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyBotHardwareInterface"), "Failed to set serial attributes");
    close(serial_fd_);
    return false;
  }

  return true;
}

void MyBotHardwareInterface::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

hardware_interface::CallbackReturn MyBotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!openSerialPort()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Serial port opened: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  closeSerialPort();
  RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Serial port closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "position", &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity", &hw_commands_[i]));
  }
  return command_interfaces;
}

bool MyBotHardwareInterface::sendSpeedCommand(double linear_x, double angular_z)
{
  unsigned char frame[12];
  frame[0] = 0x5A;
  frame[1] = 12;
  frame[2] = 0x01;
  frame[3] = 0x01;

  int16_t line_x = static_cast<int16_t>(linear_x * 1000);
  int16_t line_y = 0;
  int16_t angle_z = static_cast<int16_t>(angular_z * 1000);

  frame[4] = (line_x >> 8) & 0xFF;
  frame[5] = line_x & 0xFF;
  frame[6] = (line_y >> 8) & 0xFF;
  frame[7] = line_y & 0xFF;
  frame[8] = (angle_z >> 8) & 0xFF;
  frame[9] = angle_z & 0xFF;
  frame[10] = 0x00;
  frame[11] = calculateCRC8(frame, 11);

  int n = ::write(serial_fd_, frame, 12);
  return n == 12;
}

hardware_interface::CallbackReturn MyBotHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!sendSpeedCommand(0.0, 0.0)) {
    RCLCPP_ERROR(rclcpp::get_logger("MyBotHardwareInterface"), "Failed to send zero speed command");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyBotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  sendSpeedCommand(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool MyBotHardwareInterface::requestOdometry()
{
  unsigned char frame[8];
  frame[0] = 0x5A;
  frame[1] = 8;
  frame[2] = 0x01;
  frame[3] = 0x11;
  frame[4] = 0x00;
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = calculateCRC8(frame, 7);

  tcflush(serial_fd_, TCIFLUSH);
  int n = ::write(serial_fd_, frame, 8);
  return n == 8;
}

bool MyBotHardwareInterface::receiveOdometry(double &linear_x, double &linear_y, double &angular_z)
{
  unsigned char buffer[14];
  int total_read = 0;

  while (total_read < 14) {
    int n = ::read(serial_fd_, buffer + total_read, 14 - total_read);
    if (n <= 0) {
      return false;
    }
    total_read += n;
  }

  if (buffer[0] != 0x5A || buffer[1] != 14 || buffer[3] != 0x12) {
    return false;
  }

  unsigned char crc = calculateCRC8(buffer, 13);
  if (crc != buffer[13]) {
    return false;
  }

  int16_t line_x = (buffer[4] << 8) | buffer[5];
  int16_t line_y = (buffer[6] << 8) | buffer[7];
  int16_t angle_z = (buffer[10] << 8) | buffer[11];

  linear_x = line_x / 1000.0;
  linear_y = line_y / 1000.0;
  angular_z = angle_z / 1000.0;

  return true;
}

hardware_interface::return_type MyBotHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
  if (!requestOdometry()) {
    return hardware_interface::return_type::ERROR;
  }

  double linear_x, linear_y, angular_z;
  if (!receiveOdometry(linear_x, linear_y, angular_z)) {
    return hardware_interface::return_type::ERROR;
  }

  double vL = (linear_x - angular_z * wheel_separation_ / 2.0) / wheel_radius_;
  double vR = (linear_x + angular_z * wheel_separation_ / 2.0) / wheel_radius_;

  hw_velocities_[0] = vL;
  hw_velocities_[1] = vR;

  hw_positions_[0] += vL * period.seconds();
  hw_positions_[1] += vR * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyBotHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double vL = hw_commands_[0];
  double vR = hw_commands_[1];

  double linear_x = (vR + vL) / 2.0 * wheel_radius_;
  double angular_z = (vR - vL) * wheel_radius_ / wheel_separation_;

  if (!sendSpeedCommand(linear_x, angular_z)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_bot_hw::MyBotHardwareInterface, hardware_interface::SystemInterface)

