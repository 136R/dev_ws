#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

namespace my_bot_hw
{

/* ── Protocol constants ─────────────────────────────────────────────── */

constexpr uint8_t COMM_HEADER_1        = 0xAAu;
constexpr uint8_t COMM_HEADER_2        = 0x55u;
constexpr uint8_t COMM_TYPE_VEL_CMD    = 0x01u;
constexpr uint8_t COMM_TYPE_FEEDBACK   = 0x02u;
constexpr uint8_t COMM_VEL_CMD_LEN     = 4u;
constexpr uint8_t COMM_FEEDBACK_LEN    = 44u;
constexpr size_t  COMM_VEL_FRAME_SIZE  = 5u + COMM_VEL_CMD_LEN;     //  9 bytes
constexpr size_t  COMM_FEEDBACK_FRAME_SIZE = 5u + COMM_FEEDBACK_LEN; // 49 bytes

/* ── Checksum ───────────────────────────────────────────────────────── */

// XOR checksum: type ^ len ^ data[0] ^ ... ^ data[len-1]
uint8_t calc_xor(uint8_t type, uint8_t len, const uint8_t * data);

/* ── Velocity command frame (ROS2 → STM32) ─────────────────────────── */

// Build a TYPE=0x01 velocity command frame (9 bytes).
// left_rad_s / right_rad_s: wheel velocity targets [rad/s]
// Returns a fixed-size array: AA 55 01 04 <left_lo> <left_hi> <right_lo> <right_hi> <xor>
std::array<uint8_t, COMM_VEL_FRAME_SIZE>
build_vel_cmd(float left_rad_s, float right_rad_s);

/* ── Feedback frame (STM32 → ROS2) ─────────────────────────────────── */

struct FeedbackFrame {
    int32_t left_delta;    // encoder counts since last frame (left wheel)
    int32_t right_delta;   // encoder counts since last frame (right wheel)
    int32_t accel_mms2[3];     // linear acceleration [mm/s²]
    int32_t gyro_urad_s[3];    // angular velocity [urad/s]
    int32_t mag_nt[3];         // magnetic field [nT]
    bool    valid;
};

// Parse a 49-byte TYPE=0x02 feedback frame starting at buf[0]=0xAA.
// Returns FeedbackFrame with valid=false if checksum or format fails.
FeedbackFrame parse_feedback(const uint8_t * buf, size_t len);

}  // namespace my_bot_hw
