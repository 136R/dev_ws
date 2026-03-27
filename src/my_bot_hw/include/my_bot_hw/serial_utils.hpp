#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace my_bot_hw
{

// CRC-8/MAXIM (Dallas/Maxim 1-Wire CRC)
// poly=0x31 reflected (0x8C), init=0x00, refin=true, refout=true, xorout=0x00
uint8_t crc8_maxim(const uint8_t * data, size_t length);

// Build a complete STM32 protocol frame.
// Frame layout: 0x5A | frame_len | 0x01(ID) | func_code | data... | 0x00(reserved) | crc8
// frame_len = total frame byte count INCLUDING the 0x5A header byte = 6 + data.size()
// CRC covers all bytes from 0x5A through reserved (NOT including CRC itself).
std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t> & data);

// Parsed odometry data from a 0x0A response frame
struct OdomFrame
{
  double vx;       // linear velocity  [m/s]
  double vyaw;     // angular velocity [rad/s]
  double yaw_deg;  // cumulative heading angle [degrees]
  bool valid;
};

// Parse a received 0x0A odometry response.
// raw_frame must be the complete 12-byte frame starting with 0x5A.
// Returns OdomFrame with valid=false if CRC or format check fails.
OdomFrame parse_odom_frame(const std::vector<uint8_t> & raw_frame);

// Parsed IMU data from a 0x06 response frame
struct ImuFrame
{
  double pitch_deg{0.0};  // pitch angle [degrees]
  double roll_deg{0.0};   // roll  angle [degrees]
  double yaw_deg{0.0};    // yaw   angle [degrees]
  bool valid{false};
};

// Parse a received 0x06 IMU response.
// raw_frame must be the complete 12-byte frame starting with 0x5A.
// Returns ImuFrame with valid=false if CRC or format check fails.
ImuFrame parse_imu_frame(const std::vector<uint8_t> & raw_frame);

}  // namespace my_bot_hw
