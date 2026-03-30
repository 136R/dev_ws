#include "my_bot_hw/serial_utils.hpp"

#include <cstring>

namespace my_bot_hw
{

// ─────────────────────────── CRC-8/MAXIM ────────────────────────────────────

uint8_t crc8_maxim(const uint8_t * data, size_t length)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;  // 0x8C = reflected polynomial 0x31
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ─────────────────────────── Frame builder ───────────────────────────────────

// Frame layout:
//   Byte 0:      0x5A  (header)
//   Byte 1:      frame_len = 6 + data.size()
//   Byte 2:      0x01  (device ID)
//   Byte 3:      func_code
//   Byte 4..N-2: data
//   Byte N-1:    0x00  (reserved)
//   Byte N:      CRC-8/MAXIM over bytes[0..N-1]
std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t> & data)
{
  const uint8_t frame_len = static_cast<uint8_t>(6 + data.size());
  std::vector<uint8_t> frame;
  frame.reserve(frame_len);

  frame.push_back(0x5A);
  frame.push_back(frame_len);
  frame.push_back(0x01);          // device ID
  frame.push_back(func_code);
  for (auto b : data) {
    frame.push_back(b);
  }
  frame.push_back(0x00);          // reserved

  uint8_t crc = crc8_maxim(frame.data(), frame.size());
  frame.push_back(crc);

  return frame;
}

// ─────────────────────────── Frame parsers ───────────────────────────────────

static int16_t parse_i16_be(const uint8_t * buf, size_t offset)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1]);
}

static uint32_t parse_u32_be(const uint8_t * buf, size_t offset)
{
  return (static_cast<uint32_t>(buf[offset    ]) << 24) |
         (static_cast<uint32_t>(buf[offset + 1]) << 16) |
         (static_cast<uint32_t>(buf[offset + 2]) <<  8) |
         (static_cast<uint32_t>(buf[offset + 3])      );
}

// 0x04 Wheel RPM frame — 14 bytes total, 8 bytes data
// [0x5A][0x0E][0x01][0x04][ts(4)][L×10(2)][R×10(2)][0x00][CRC]
WheelRpmFrame parse_wheel_rpm_frame(const std::vector<uint8_t> & raw)
{
  WheelRpmFrame r;

  if (raw.size() != 14) { return r; }
  if (raw[0] != 0x5A)   { return r; }
  if (raw[1] != 14)     { return r; }
  if (raw[2] != 0x01)   { return r; }
  if (raw[3] != 0x04)   { return r; }

  uint8_t expected_crc = crc8_maxim(raw.data(), 13);
  if (raw[13] != expected_crc) { return r; }

  r.timestamp_us = parse_u32_be(raw.data(), 4);
  r.left_rpm     = parse_i16_be(raw.data(), 8) / 10.0;
  r.right_rpm    = parse_i16_be(raw.data(), 10) / 10.0;
  r.valid        = true;
  return r;
}

// 0x06 IMU frame — 36 bytes total, 30 bytes data
// [0x5A][0x24][0x01][0x06][ts(4)][acc(6)][gyro(6)][rpy(6)][quat(8)][0x00][CRC]
ImuRawFrame parse_imu_raw_frame(const std::vector<uint8_t> & raw)
{
  ImuRawFrame r;

  if (raw.size() != 36) { return r; }
  if (raw[0] != 0x5A)   { return r; }
  if (raw[1] != 36)     { return r; }
  if (raw[2] != 0x01)   { return r; }
  if (raw[3] != 0x06)   { return r; }

  uint8_t expected_crc = crc8_maxim(raw.data(), 35);
  if (raw[35] != expected_crc) { return r; }

  size_t off = 4;
  r.timestamp_us = parse_u32_be(raw.data(), off); off += 4;

  for (int i = 0; i < 3; i++) { r.acc[i]  = parse_i16_be(raw.data(), off); off += 2; }
  for (int i = 0; i < 3; i++) { r.gyro[i] = parse_i16_be(raw.data(), off); off += 2; }

  r.roll  = parse_i16_be(raw.data(), off); off += 2;
  r.pitch = parse_i16_be(raw.data(), off); off += 2;
  r.yaw   = parse_i16_be(raw.data(), off); off += 2;

  for (int i = 0; i < 4; i++) { r.quat[i] = parse_i16_be(raw.data(), off); off += 2; }

  r.valid = true;
  return r;
}

}  // namespace my_bot_hw
