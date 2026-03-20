#ifndef MY_BOT_HW__STM32_COMM_HPP_
#define MY_BOT_HW__STM32_COMM_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace my_bot_hw
{

// CRC8 calculation for STM32 communication protocol
// Algorithm: LSB-first, Polynomial 0x8C, Initial value 0, no final XOR
// Based on vendor firmware: docs/vendor/Src/APP/Communication.c

// Calculate CRC8 for a vector of bytes
uint8_t crc8_calculate(const std::vector<uint8_t> & data);

// Calculate CRC8 for a raw byte buffer
uint8_t crc8_calculate(const uint8_t * data, size_t length);

// Protocol frame packing functions
// Frame format: [0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]

// Build a protocol frame with given command and data payload
// cmd: command code (e.g., 0x01, 0x11, 0x13)
// data: data bytes (for no-data commands, pass {0x00} as placeholder)
std::vector<uint8_t> pack_frame(uint8_t cmd, const std::vector<uint8_t> & data);

// Build CMD 0x01: velocity command
// vx: linear velocity X (m/s)
// vy: linear velocity Y (m/s), typically 0 for differential drive
// wz: angular velocity Z (rad/s)
std::vector<uint8_t> build_cmd_velocity(float vx, float vy, float wz);

// Build CMD 0x11: request odometry data
std::vector<uint8_t> build_cmd_request_odom();

// Build CMD 0x13: request raw IMU data
std::vector<uint8_t> build_cmd_request_imu();

}  // namespace my_bot_hw

#endif  // MY_BOT_HW__STM32_COMM_HPP_
