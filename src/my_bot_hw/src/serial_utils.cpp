#include "my_bot_hw/serial_utils.hpp"

#include <cstring>
#include <stdexcept>

namespace my_bot_hw
{

// CRC-8/MAXIM: poly=0x31 reflected, init=0x00
// Bit-by-bit implementation using reflected polynomial 0x8C
uint8_t crc8_maxim(const uint8_t * data, size_t length)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;  // 0x8C = reflected 0x31
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Frame layout (verified against protocol examples):
//   Byte 0: 0x5A (header, fixed)
//   Byte 1: frame_len = TOTAL frame byte count including 0x5A = 6 + data.size()
//   Byte 2: 0x01 (device ID)
//   Byte 3: func_code
//   Byte 4..N-2: data (0..250 bytes)
//   Byte N-1: 0x00 (reserved)
//   Byte N:   CRC-8/MAXIM over bytes[0..N-1] (from 0x5A through reserved)
//
// Verification:
//   Query odometry (0x09, no data): frame_len=6, total=6 bytes
//   5A 06 01 09 00 38  →  CRC covers {5A,06,01,09,00} = 0x38 ✓
//   Velocity 0.5m/s (0x01, 6 data bytes): frame_len=12=0x0C, total=12 bytes
//   5A 0C 01 01 01 F4 00 00 00 00 00 56  ✓
std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t> & data)
{
  const uint8_t frame_len = static_cast<uint8_t>(6 + data.size());
  std::vector<uint8_t> frame;
  frame.reserve(frame_len);

  frame.push_back(0x5A);          // header
  frame.push_back(frame_len);     // total frame length (including 0x5A)
  frame.push_back(0x01);          // device ID
  frame.push_back(func_code);     // function code
  for (auto b : data) {
    frame.push_back(b);
  }
  frame.push_back(0x00);          // reserved

  // CRC covers from 0x5A header (index 0) through reserved byte (index N-1)
  uint8_t crc = crc8_maxim(frame.data(), frame.size());
  frame.push_back(crc);

  return frame;
}

OdomFrame parse_odom_frame(const std::vector<uint8_t> & raw_frame)
{
  OdomFrame result{0.0, 0.0, 0.0, false};

  // 0x0A response: 12 bytes total
  // 5A | 0C | 01 | 0A | vx_lo vx_hi | vyaw_lo vyaw_hi | yaw_lo yaw_hi | 00 | crc
  if (raw_frame.size() != 12) {
    return result;
  }
  if (raw_frame[0] != 0x5A) {
    return result;
  }
  if (raw_frame[1] != 0x0C) {  // frame_len = 12
    return result;
  }
  if (raw_frame[3] != 0x0A) {  // function code
    return result;
  }

  // Validate CRC: covers bytes[0..10] (0x5A header through reserved)
  uint8_t expected_crc = crc8_maxim(raw_frame.data(), raw_frame.size() - 1);
  if (raw_frame[11] != expected_crc) {
    return result;
  }

  // Parse int16 values (big-endian: MSB first, LSB second)
  // Protocol doc (底盘-ROS通讯协议.txt) and STM32 Communication.c vGetOdmInfoCommand:
  //   Byte1-2 (frame[4:5]): X   = vx      × 1000  (linear velocity m/s)
  //   Byte3-4 (frame[6:7]): Yaw = yaw_deg × 100   (IMU Euler heading °, from xImu.pfEuler[2])
  //   Byte5-6 (frame[8:9]): Z   = vyaw    × 1000  (angular velocity rad/s, from encoder kinematics)
  auto parse_int16_be = [](uint8_t msb, uint8_t lsb) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  };

  result.vx      = parse_int16_be(raw_frame[4], raw_frame[5]) / 1000.0;  // m/s
  result.yaw_deg = parse_int16_be(raw_frame[6], raw_frame[7]) / 100.0;   // degrees
  result.vyaw    = parse_int16_be(raw_frame[8], raw_frame[9]) / 1000.0;  // rad/s
  result.valid   = true;

  return result;
}

ImuFrame parse_imu_frame(const std::vector<uint8_t> & raw_frame)
{
  ImuFrame result{0.0, 0.0, 0.0, false};

  // 0x06 response: 12 bytes total
  // 5A | 0C | 01 | 06 | pitch_H pitch_L | roll_H roll_L | yaw_H yaw_L | 00 | crc
  if (raw_frame.size() != 12) {
    return result;
  }
  if (raw_frame[0] != 0x5A) {
    return result;
  }
  if (raw_frame[1] != 0x0C) {  // frame_len = 12
    return result;
  }
  if (raw_frame[3] != 0x06) {  // function code
    return result;
  }

  // Validate CRC: covers bytes[0..10] (0x5A header through reserved)
  uint8_t expected_crc = crc8_maxim(raw_frame.data(), raw_frame.size() - 1);
  if (raw_frame[11] != expected_crc) {
    return result;
  }

  // Parse int16 values (big-endian: MSB first)
  // Units: degrees × 1000 (int16)
  auto parse_int16_be = [](uint8_t msb, uint8_t lsb) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  };

  int16_t raw_pitch = parse_int16_be(raw_frame[4], raw_frame[5]);
  int16_t raw_roll  = parse_int16_be(raw_frame[6], raw_frame[7]);
  int16_t raw_yaw   = parse_int16_be(raw_frame[8], raw_frame[9]);

  result.pitch_deg = raw_pitch / 1000.0;  // degrees
  result.roll_deg  = raw_roll  / 1000.0;  // degrees
  result.yaw_deg   = raw_yaw   / 1000.0;  // degrees
  result.valid = true;

  return result;
}

}  // namespace my_bot_hw
