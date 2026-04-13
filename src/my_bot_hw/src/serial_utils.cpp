#include "my_bot_hw/serial_utils.hpp"

#include <algorithm>
#include <cmath>

namespace my_bot_hw
{

uint8_t calc_xor(uint8_t type, uint8_t len, const uint8_t * data)
{
    uint8_t x = type ^ len;
    for (uint8_t i = 0; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

std::array<uint8_t, COMM_VEL_FRAME_SIZE>
build_vel_cmd(float left_rad_s, float right_rad_s)
{
    // Clamp to int16 range (±32.767 rad/s, well above wheel_max ~27.2 rad/s)
    auto to_mrad = [](float v) -> int16_t {
        float scaled = v * 1000.0f;
        scaled = std::clamp(scaled, static_cast<float>(INT16_MIN), static_cast<float>(INT16_MAX));
        return static_cast<int16_t>(scaled);
    };

    int16_t left_mrad  = to_mrad(left_rad_s);
    int16_t right_mrad = to_mrad(right_rad_s);

    std::array<uint8_t, COMM_VEL_FRAME_SIZE> frame{};
    frame[0] = COMM_HEADER_1;
    frame[1] = COMM_HEADER_2;
    frame[2] = COMM_TYPE_VEL_CMD;
    frame[3] = COMM_VEL_CMD_LEN;
    // Little-endian int16
    frame[4] = static_cast<uint8_t>( left_mrad        & 0xFF);
    frame[5] = static_cast<uint8_t>((left_mrad  >> 8) & 0xFF);
    frame[6] = static_cast<uint8_t>( right_mrad       & 0xFF);
    frame[7] = static_cast<uint8_t>((right_mrad >> 8) & 0xFF);
    frame[8] = calc_xor(frame[2], frame[3], &frame[4]);
    return frame;
}

FeedbackFrame parse_feedback(const uint8_t * buf, size_t len)
{
    FeedbackFrame result{};
    result.valid = false;

    if (len < COMM_FEEDBACK_FRAME_SIZE) return result;
    if (buf[0] != COMM_HEADER_1)        return result;
    if (buf[1] != COMM_HEADER_2)        return result;
    if (buf[2] != COMM_TYPE_FEEDBACK)   return result;
    if (buf[3] != COMM_FEEDBACK_LEN)    return result;

    const uint8_t * data = &buf[4];
    uint8_t expected_xor = calc_xor(buf[2], buf[3], data);
    if (buf[4 + COMM_FEEDBACK_LEN] != expected_xor) return result;

    // Unpack little-endian int32
    auto unpack_i32 = [](const uint8_t * p) -> int32_t {
        return static_cast<int32_t>(
            static_cast<uint32_t>(p[0])        |
            (static_cast<uint32_t>(p[1]) << 8) |
            (static_cast<uint32_t>(p[2]) << 16)|
            (static_cast<uint32_t>(p[3]) << 24));
    };

    result.left_delta   = unpack_i32(&data[0]);
    result.right_delta  = unpack_i32(&data[4]);
    result.accel_mms2[0] = unpack_i32(&data[8]);
    result.accel_mms2[1] = unpack_i32(&data[12]);
    result.accel_mms2[2] = unpack_i32(&data[16]);
    result.gyro_urad_s[0] = unpack_i32(&data[20]);
    result.gyro_urad_s[1] = unpack_i32(&data[24]);
    result.gyro_urad_s[2] = unpack_i32(&data[28]);
    result.mag_nt[0] = unpack_i32(&data[32]);
    result.mag_nt[1] = unpack_i32(&data[36]);
    result.mag_nt[2] = unpack_i32(&data[40]);
    result.valid        = true;
    return result;
}

}  // namespace my_bot_hw
