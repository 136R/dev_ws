#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace my_bot_hw
{

// CRC-8/MAXIM (Dallas/Maxim 1-Wire CRC)
// poly=0x31 reflected (0x8C), init=0x00
// Test vector: {0x5A,0x06,0x01,0x09,0x00} → 0x38
uint8_t crc8_maxim(const uint8_t * data, size_t length);

// Build a complete STM32 protocol frame.
// Frame layout: 0x5A | frame_len | 0x01(ID) | func_code | data... | 0x00(reserved) | crc8
// frame_len = total frame byte count INCLUDING the 0x5A header = 6 + data.size()
// CRC covers all bytes from 0x5A through reserved (NOT including CRC itself).
std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t> & data);

// ── 0x04 Wheel RPM broadcast frame ────────────────────────────────────────
// Frame: 0x5A | 0x0E | 0x01 | 0x04 | timestamp_us(4) | L_rpm×10(2) | R_rpm×10(2) | 0x00 | CRC
// Total: 14 bytes, data: 8 bytes
struct WheelRpmFrame
{
  uint32_t timestamp_us{0};   // hardware timestamp [µs], wraps ~59s
  double   left_rpm{0.0};     // left  wheel speed [RPM]
  double   right_rpm{0.0};    // right wheel speed [RPM]
  bool     valid{false};
};

// Parse a received 0x04 broadcast frame (14 bytes starting with 0x5A).
WheelRpmFrame parse_wheel_rpm_frame(const std::vector<uint8_t> & raw_frame);

// ── 0x06 IMU broadcast frame ───────────────────────────────────────────────
// Frame: 0x5A | 0x24 | 0x01 | 0x06 | timestamp_us(4) |
//        acc[3](6) | gyro[3](6) | roll,pitch,yaw(6) | quat[4](8) | 0x00 | CRC
// Total: 36 bytes, data: 30 bytes
// All int16 values are raw sensor units; caller applies scale factors from Config.h.
struct ImuRawFrame
{
  uint32_t timestamp_us{0};
  int16_t  acc[3]{};           // raw accelerometer X/Y/Z
  int16_t  gyro[3]{};          // raw gyroscope     X/Y/Z
  int16_t  roll{0};            // raw roll  angle
  int16_t  pitch{0};           // raw pitch angle
  int16_t  yaw{0};             // raw yaw   angle
  int16_t  quat[4]{};          // raw quaternion W/X/Y/Z
  bool     valid{false};
};

// Parse a received 0x06 broadcast frame (36 bytes starting with 0x5A).
ImuRawFrame parse_imu_raw_frame(const std::vector<uint8_t> & raw_frame);

}  // namespace my_bot_hw
