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
constexpr uint8_t COMM_TYPE_BATTERY    = 0x03u;
constexpr uint8_t COMM_VEL_CMD_LEN     = 4u;
constexpr uint8_t COMM_FEEDBACK_LEN    = 36u;  // 2×int32 encoder + accel×3 + gyro×3 + yaw×1
constexpr uint8_t COMM_BATTERY_LEN     = 8u;   // voltage_mv + current_ma + flags + proto_ver + reserved
constexpr size_t  COMM_VEL_FRAME_SIZE  = 5u + COMM_VEL_CMD_LEN;     //  9 bytes
constexpr size_t  COMM_FEEDBACK_FRAME_SIZE = 5u + COMM_FEEDBACK_LEN; // 41 bytes
constexpr size_t  COMM_BATTERY_FRAME_SIZE  = 5u + COMM_BATTERY_LEN;  // 13 bytes

// Smallest prefix that lets the scanner decide what a frame is:
// header(2) + type(1) + len(1) + at least one more byte.
constexpr size_t COMM_MIN_FRAME_PREFIX = 5u;

// Payload version expected in the 0x03 frame.  Must match COMM_PROTO_VER in
// the firmware's comm_protocol.h.  git tracks the sources; this checks that
// the binary actually flashed on the board matches this build — it is what
// turns "edited ROS, forgot to reflash" into a loud warning.
constexpr uint8_t COMM_PROTO_VER = 1u;

// Battery flag bits (0x03 payload byte 4)
constexpr uint8_t BATTERY_FLAG_VOLTAGE_VALID = 0x01u;
constexpr uint8_t BATTERY_FLAG_CURRENT_VALID = 0x02u;
constexpr uint8_t BATTERY_FLAG_VREFINT_OK    = 0x04u;

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
    int32_t left_delta;        // encoder counts since last frame (left wheel)
    int32_t right_delta;       // encoder counts since last frame (right wheel)
    int32_t accel_mms2[3];     // linear acceleration [mm/s²]
    int32_t gyro_urad_s[3];    // angular velocity [urad/s]
    int32_t yaw_mdeg;          // Fusion AHRS Euler yaw [millidegrees]
    bool    valid;
};

// Parse a 41-byte TYPE=0x02 feedback frame starting at buf[0]=0xAA.
// Returns FeedbackFrame with valid=false if checksum or format fails.
FeedbackFrame parse_feedback(const uint8_t * buf, size_t len);

/* ── Battery frame (STM32 → ROS2) ──────────────────────────────────── */

struct BatteryFrame {
    uint16_t voltage_mv;   // battery terminal voltage [mV]
    int16_t  current_ma;   // always 0 in proto v1 (no current sense)
    uint8_t  flags;        // BATTERY_FLAG_*
    uint8_t  proto_ver;    // firmware's COMM_PROTO_VER
    bool     valid;

    bool voltage_valid() const { return (flags & BATTERY_FLAG_VOLTAGE_VALID) != 0u; }
    bool vrefint_ok()    const { return (flags & BATTERY_FLAG_VREFINT_OK) != 0u; }
};

// Parse a 13-byte TYPE=0x03 battery frame starting at buf[0]=0xAA.
// Returns BatteryFrame with valid=false if checksum or format fails.
// Note: valid=true only means the frame arrived intact — proto_ver and the
// flag bits still have to be checked by the caller.
BatteryFrame parse_battery(const uint8_t * buf, size_t len);

/* ── Frame dispatch ─────────────────────────────────────────────────── */

// Payload length for a STM32→ROS2 frame type, or 0 if the type is unknown.
// The stream scanner uses this to tell "this frame has not arrived in full
// yet, wait for more bytes" apart from "this is not a frame header at all".
// Getting that distinction wrong drops odometry: advancing by one byte over
// a partially received 0x02 consumes its header and loses the whole frame.
size_t frame_payload_len(uint8_t type);

}  // namespace my_bot_hw
