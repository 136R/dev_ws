#include "my_bot_hw/stm32_comm.hpp"

namespace my_bot_hw
{

uint8_t crc8_calculate(const uint8_t * data, size_t length)
{
  uint8_t crc = 0;
  uint8_t i;

  for (size_t idx = 0; idx < length; idx++) {
    crc ^= data[idx];
    for (i = 0; i < 8; i++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8c;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

uint8_t crc8_calculate(const std::vector<uint8_t> & data)
{
  if (data.empty()) {
    return 0;
  }
  return crc8_calculate(data.data(), data.size());
}

std::vector<uint8_t> pack_frame(uint8_t cmd, const std::vector<uint8_t> & data)
{
  // Frame format: [0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]
  // Total length = header(1) + len(1) + device_id(1) + cmd(1) + data + reserved(1) + crc(1)
  size_t total_len = 6 + data.size();
  std::vector<uint8_t> frame(total_len);

  frame[0] = 0x5A;
  frame[1] = static_cast<uint8_t>(total_len);
  frame[2] = 0x01;  // device ID
  frame[3] = cmd;

  // Copy data bytes
  for (size_t i = 0; i < data.size(); i++) {
    frame[4 + i] = data[i];
  }

  // Reserved byte (always 0x00, placed before CRC)
  frame[total_len - 2] = 0x00;

  // Calculate and place CRC8 over all bytes except the CRC byte itself
  frame[total_len - 1] = crc8_calculate(frame.data(), total_len - 1);

  return frame;
}

std::vector<uint8_t> build_cmd_velocity(float vx, float vy, float wz)
{
  // Encode velocities as int16_t with factor 1000
  int16_t ivx = static_cast<int16_t>(vx * 1000.0f);
  int16_t ivy = static_cast<int16_t>(vy * 1000.0f);
  int16_t iwz = static_cast<int16_t>(wz * 1000.0f);

  // Pack as big-endian: high byte first, then low byte
  std::vector<uint8_t> data = {
    static_cast<uint8_t>((ivx >> 8) & 0xFF),
    static_cast<uint8_t>(ivx & 0xFF),
    static_cast<uint8_t>((ivy >> 8) & 0xFF),
    static_cast<uint8_t>(ivy & 0xFF),
    static_cast<uint8_t>((iwz >> 8) & 0xFF),
    static_cast<uint8_t>(iwz & 0xFF),
  };

  return pack_frame(0x01, data);
}

std::vector<uint8_t> build_cmd_request_odom()
{
  // CMD 0x11: request odometry
  // Frame has one placeholder data byte to ensure total length = 7
  return pack_frame(0x11, {0x00});
}

std::vector<uint8_t> build_cmd_request_imu()
{
  // CMD 0x13: request raw IMU data
  // Frame has one placeholder data byte to ensure total length = 7
  return pack_frame(0x13, {0x00});
}

}  // namespace my_bot_hw
