#!/usr/bin/env python3
"""
USART1 通信测试脚本
测试 OrangePi <-> STM32 串口协议

协议格式:
  下行 0x01: [0x5A][0x0A][0x01][0x01][L_H][L_L][R_H][R_L][0x00][CRC8]
  上行 0x04: [0x5A][0x0E][0x01][0x04][ts(4)][L×10(2)][R×10(2)][0x00][CRC]  14B
  上行 0x06: [0x5A][0x24][0x01][0x06][ts(4)][acc(6)][gyro(6)][rpy(6)][quat(8)][0x00][CRC]  36B

用法:
  python3 test_usart1.py                    # 只监听广播，不发送指令
  python3 test_usart1.py --left 30 --right 30   # 发送目标转速 30 RPM
  python3 test_usart1.py --port /dev/ttyS7 --left 0 --right 0  # 停止
"""

import serial
import struct
import time
import argparse
import sys

# ── 协议常量 ──────────────────────────────────────────────────
FRAME_HEADER = 0x5A
DEVICE_ID    = 0x01
CRC8_POLY    = 0x8C   # CRC8-MAXIM (反射多项式)

# IMU 缩放系数（与 Config.h 一致）
IMU_GYRO_SCALE  = 2048.0 / 32768.0 * 3.14159265 / 180.0   # → rad/s
IMU_ACCEL_SCALE = 9.8 * 16.0 / 32768.0                     # → m/s²
IMU_QUAT_SCALE  = 0.0001
IMU_ANGLE_SCALE = 0.01   # → 度


def crc8_maxim(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ CRC8_POLY
            else:
                crc >>= 1
    return crc & 0xFF


def build_cmd_01(left_rpm: float, right_rpm: float) -> bytes:
    """构建 0x01 设置轮速帧（10字节）"""
    l_x10 = int(left_rpm  * 10)
    r_x10 = int(right_rpm * 10)
    # 帧体（不含CRC）
    body = struct.pack('>BBBBhhB',
                       FRAME_HEADER,   # 0x5A
                       10,             # frame_len
                       DEVICE_ID,      # 0x01
                       0x01,           # func code
                       l_x10,          # left  RPM×10 int16 大端
                       r_x10,          # right RPM×10 int16 大端
                       0x00)           # reserved
    crc = crc8_maxim(body)
    return body + bytes([crc])


def parse_frame_04(payload: bytes):
    """解析 0x04 轮速帧 payload（去掉头6字节后的数据部分）"""
    # data: ts(4) + L×10(2) + R×10(2) = 8 字节
    ts_us, l_x10, r_x10 = struct.unpack('>Ihh', payload[:8])
    return {
        'ts_us':     ts_us,
        'left_rpm':  l_x10 / 10.0,
        'right_rpm': r_x10 / 10.0,
    }


def parse_frame_06(payload: bytes):
    """解析 0x06 IMU 帧 payload"""
    # ts(4) + acc(6) + gyro(6) + rpy(6) + quat(8) = 30 字节
    ts_us = struct.unpack('>I', payload[:4])[0]
    ax, ay, az = struct.unpack('>hhh', payload[4:10])
    gx, gy, gz = struct.unpack('>hhh', payload[10:16])
    roll, pitch, yaw = struct.unpack('>hhh', payload[16:22])
    qw, qx, qy, qz = struct.unpack('>hhhh', payload[22:30])
    return {
        'ts_us': ts_us,
        'accel': [ax * IMU_ACCEL_SCALE, ay * IMU_ACCEL_SCALE, az * IMU_ACCEL_SCALE],
        'gyro':  [gx * IMU_GYRO_SCALE,  gy * IMU_GYRO_SCALE,  gz * IMU_GYRO_SCALE],
        'roll':  roll  * IMU_ANGLE_SCALE,
        'pitch': pitch * IMU_ANGLE_SCALE,
        'yaw':   yaw   * IMU_ANGLE_SCALE,
        'quat':  [qw * IMU_QUAT_SCALE, qx * IMU_QUAT_SCALE,
                  qy * IMU_QUAT_SCALE, qz * IMU_QUAT_SCALE],
    }


def recv_frame(ser: serial.Serial, timeout_s: float = 1.0) -> tuple:
    """
    从串口读取一帧，返回 (func_code, payload_bytes) 或 (None, None)。
    """
    deadline = time.time() + timeout_s
    buf = bytearray()

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b

        # 等待至少 2 字节（header + len）
        if len(buf) < 2:
            if buf[0] != FRAME_HEADER:
                buf.clear()
            continue

        if buf[0] != FRAME_HEADER:
            buf.pop(0)
            continue

        frame_len = buf[1]
        if frame_len < 6 or frame_len > 64:
            buf.clear()
            continue

        # 读完整帧
        while len(buf) < frame_len and time.time() < deadline:
            chunk = ser.read(frame_len - len(buf))
            if chunk:
                buf += chunk

        if len(buf) < frame_len:
            return None, None

        frame = bytes(buf[:frame_len])

        # 验证 Device ID
        if frame[2] != DEVICE_ID:
            buf = buf[1:]
            continue

        # 验证 CRC
        expected = crc8_maxim(frame[:-1])
        if frame[-1] != expected:
            print(f"  [CRC错误] 期望 0x{expected:02X} 实际 0x{frame[-1]:02X}")
            buf = buf[1:]
            continue

        func_code = frame[3]
        # payload = 去掉 header(1)+len(1)+devid(1)+func(1)+reserved(1)+crc(1) = 6字节
        payload = frame[4:-2]   # [4 .. frame_len-3]，不含 reserved 和 crc
        return func_code, payload

    return None, None


def main():
    parser = argparse.ArgumentParser(description='STM32 USART1 通信测试')
    parser.add_argument('--port',  default='/dev/ttyS7', help='串口设备')
    parser.add_argument('--baud',  type=int, default=460800, help='波特率')
    parser.add_argument('--left',  type=float, default=None, help='左轮目标 RPM')
    parser.add_argument('--right', type=float, default=None, help='右轮目标 RPM')
    parser.add_argument('--count', type=int, default=20, help='接收帧数后退出（0=无限）')
    args = parser.parse_args()

    print(f"打开串口 {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"[错误] 无法打开串口: {e}")
        sys.exit(1)

    time.sleep(0.1)
    ser.reset_input_buffer()

    # 发送轮速指令
    if args.left is not None and args.right is not None:
        cmd = build_cmd_01(args.left, args.right)
        ser.write(cmd)
        print(f"发送 0x01 指令: 左={args.left} RPM, 右={args.right} RPM")
        print(f"  原始字节: {cmd.hex(' ').upper()}")

    # 验证 CRC8 测试向量
    test_vec = bytes([0x5A, 0x06, 0x01, 0x09, 0x00])
    assert crc8_maxim(test_vec) == 0x38, "CRC8 测试向量错误！"

    print(f"\n监听广播帧（STM32 @ 50Hz）...\n{'─'*60}")

    received = 0
    last_04_ts = None

    while args.count == 0 or received < args.count:
        func, payload = recv_frame(ser, timeout_s=0.5)

        if func is None:
            print("  [超时] 未收到帧，检查接线和波特率")
            continue

        received += 1

        if func == 0x04:
            d = parse_frame_04(payload)
            dt_ms = ''
            if last_04_ts is not None:
                dt_us = (d['ts_us'] - last_04_ts) & 0xFFFFFFFF
                dt_ms = f"  Δt={dt_us/1000:.1f}ms"
            last_04_ts = d['ts_us']
            print(f"[0x04 轮速] ts={d['ts_us']:>10}µs  "
                  f"L={d['left_rpm']:+7.1f} RPM  R={d['right_rpm']:+7.1f} RPM{dt_ms}")

        elif func == 0x06:
            d = parse_frame_06(payload)
            print(f"[0x06 IMU ] "
                  f"acc=[{d['accel'][0]:+6.3f},{d['accel'][1]:+6.3f},{d['accel'][2]:+6.3f}] m/s²  "
                  f"gyro=[{d['gyro'][0]:+6.3f},{d['gyro'][1]:+6.3f},{d['gyro'][2]:+6.3f}] rad/s  "
                  f"RPY=({d['roll']:+6.2f},{d['pitch']:+6.2f},{d['yaw']:+7.2f})°")
        else:
            print(f"[未知帧 0x{func:02X}] payload={payload.hex()}")

    ser.close()
    print(f"\n完成，共收到 {received} 帧。")


if __name__ == '__main__':
    main()
