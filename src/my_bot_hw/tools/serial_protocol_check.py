#!/usr/bin/env python3
"""STM32 serial protocol acceptance helper.

Supports:
- listen: monitor 0x02 feedback frames and report frequency / checksum errors
- send: repeatedly send 0x01 velocity commands
- watchdog-test: drive for a short duration, then stop sending and observe feedback

范例：

先监听反馈帧，确认 STM32 正在稳定发送 0x02：

python3 /home/orangepi/dev_ws/src/my_bot_hw/tools/serial_protocol_check.py \
  --port /dev/ttyS7 --baudrate 115200 listen
看两点：

summary 里的 rate 应接近 50 Hz
bad_frames 应长期保持 0

---

手动发速度命令，先低速、最好架空轮子：

python3 /home/orangepi/dev_ws/src/my_bot_hw/tools/serial_protocol_check.py \
  --port /dev/ttyS7 --baudrate 115200 send \
  --left 1.0 --right 1.0 --rate 50 --duration 2 --send-zero-on-exit

通过标准：

电机按预期启动
左右方向正确
退出时能收到零速并停下

---

测 watchdog，脚本会先发命令，再停止发送并观察编码器增量何时接近 0：

python3 /home/orangepi/dev_ws/src/my_bot_hw/tools/serial_protocol_check.py \
  --port /dev/ttyS7 --baudrate 115200 watchdog-test \
  --left 6.15 --right 6.15 --rate 50 --drive-time 5.0 --observe-time 5.5
通过标准：

停止发送后，约 0.5s + 机械滑行时间 内，left_delta/right_delta 下降到接近 0
脚本最后会打印 watchdog result
"""

from __future__ import annotations

import argparse
import math
import os
import select
import signal
import struct
import sys
import termios
import time
from dataclasses import dataclass
from typing import Optional


COMM_HEADER_1 = 0xAA
COMM_HEADER_2 = 0x55
COMM_TYPE_VEL_CMD = 0x01
COMM_TYPE_FEEDBACK = 0x02
COMM_VEL_CMD_LEN = 4
COMM_FEEDBACK_LEN = 44
COMM_VEL_FRAME_SIZE = 9
COMM_FEEDBACK_FRAME_SIZE = 49
COUNTS_PER_REV = 68000.0

RUNNING = True


def handle_sigint(_signum, _frame) -> None:
    global RUNNING
    RUNNING = False


def calc_xor(msg_type: int, payload: bytes) -> int:
    value = msg_type ^ len(payload)
    for byte in payload:
        value ^= byte
    return value


def build_vel_cmd(left_rad_s: float, right_rad_s: float) -> bytes:
    left_mrad = max(min(int(round(left_rad_s * 1000.0)), 32767), -32768)
    right_mrad = max(min(int(round(right_rad_s * 1000.0)), 32767), -32768)
    payload = struct.pack("<hh", left_mrad, right_mrad)
    return bytes(
        [COMM_HEADER_1, COMM_HEADER_2, COMM_TYPE_VEL_CMD, COMM_VEL_CMD_LEN]
    ) + payload + bytes([calc_xor(COMM_TYPE_VEL_CMD, payload)])


@dataclass
class FeedbackFrame:
    left_delta: int
    right_delta: int
    accel_mms2: tuple[int, int, int]
    gyro_urad_s: tuple[int, int, int]
    mag_nt: tuple[int, int, int]

    @property
    def left_rad(self) -> float:
        return self.left_delta / COUNTS_PER_REV * 2.0 * math.pi

    @property
    def right_rad(self) -> float:
        return self.right_delta / COUNTS_PER_REV * 2.0 * math.pi


class SerialPort:
    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.fd: Optional[int] = None

    def __enter__(self) -> "SerialPort":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        baud = baudrate_to_termios(self.baudrate)
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read(self, size: int = 256, timeout: float = 0.1) -> bytes:
        if self.fd is None:
            raise RuntimeError("serial port is not open")
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if not readable:
            return b""
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write_all(self, data: bytes, timeout: float = 0.2) -> None:
        if self.fd is None:
            raise RuntimeError("serial port is not open")
        deadline = time.monotonic() + timeout
        sent = 0
        while sent < len(data):
            try:
                n = os.write(self.fd, data[sent:])
            except BlockingIOError:
                n = 0
            if n > 0:
                sent += n
                continue
            if time.monotonic() > deadline:
                raise TimeoutError(f"serial write timeout after sending {sent}/{len(data)} bytes")
            _, writable, _ = select.select([], [self.fd], [], 0.01)
            if not writable:
                time.sleep(0.001)


def baudrate_to_termios(baudrate: int) -> int:
    mapping = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
    }
    if baudrate not in mapping:
        raise ValueError(f"unsupported baudrate: {baudrate}")
    return mapping[baudrate]


class FeedbackParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0
        self.total_bytes = 0

    def feed(self, chunk: bytes) -> list[FeedbackFrame]:
        self.buffer.extend(chunk)
        self.total_bytes += len(chunk)
        frames: list[FeedbackFrame] = []

        while len(self.buffer) >= COMM_FEEDBACK_FRAME_SIZE:
            if self.buffer[0] != COMM_HEADER_1 or self.buffer[1] != COMM_HEADER_2:
                del self.buffer[0]
                continue

            frame = self.buffer[:COMM_FEEDBACK_FRAME_SIZE]
            if frame[2] != COMM_TYPE_FEEDBACK or frame[3] != COMM_FEEDBACK_LEN:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            payload = frame[4:4 + COMM_FEEDBACK_LEN]
            xor_value = calc_xor(COMM_TYPE_FEEDBACK, payload)
            if frame[4 + COMM_FEEDBACK_LEN] != xor_value:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            unpacked = struct.unpack("<" + "i" * 11, payload)
            frames.append(
                FeedbackFrame(
                    left_delta=unpacked[0],
                    right_delta=unpacked[1],
                    accel_mms2=(unpacked[2], unpacked[3], unpacked[4]),
                    gyro_urad_s=(unpacked[5], unpacked[6], unpacked[7]),
                    mag_nt=(unpacked[8], unpacked[9], unpacked[10]),
                )
            )
            del self.buffer[:COMM_FEEDBACK_FRAME_SIZE]

        return frames


def format_frame(frame: FeedbackFrame) -> str:
    return (
        f"left_delta={frame.left_delta:6d} ({frame.left_rad:+.4f} rad)  "
        f"right_delta={frame.right_delta:6d} ({frame.right_rad:+.4f} rad)  "
        f"acc_z={frame.accel_mms2[2] / 1000.0:+6.3f} m/s^2  "
        f"gyro_z={frame.gyro_urad_s[2] * 1e-6:+7.4f} rad/s  "
        f"mag=({frame.mag_nt[0] * 1e-3:+6.1f},"
        f"{frame.mag_nt[1] * 1e-3:+6.1f},"
        f"{frame.mag_nt[2] * 1e-3:+6.1f}) uT"
    )


def cmd_listen(args: argparse.Namespace) -> int:
    parser = FeedbackParser()
    frame_count = 0
    start = time.monotonic()
    last_report = start

    with SerialPort(args.port, args.baudrate) as serial_port:
        print(f"listening on {args.port} @ {args.baudrate} bps, Ctrl+C to stop")
        while RUNNING:
            chunk = serial_port.read(timeout=0.2)
            if chunk:
                frames = parser.feed(chunk)
                for frame in frames:
                    frame_count += 1
                    if args.print_every <= 1 or frame_count % args.print_every == 0:
                        print(f"[{frame_count:06d}] {format_frame(frame)}")

            now = time.monotonic()
            if now - last_report >= args.report_interval:
                elapsed = now - start
                hz = frame_count / elapsed if elapsed > 0.0 else 0.0
                print(
                    f"summary: frames={frame_count}  bad_frames={parser.bad_frames}  "
                    f"rate={hz:.2f} Hz  buffered={len(parser.buffer)} bytes"
                )
                last_report = now

    return 0


def cmd_send(args: argparse.Namespace) -> int:
    frame = build_vel_cmd(args.left, args.right)
    interval = 1.0 / args.rate
    deadline = time.monotonic() + args.duration if args.duration > 0.0 else None
    sent = 0

    with SerialPort(args.port, args.baudrate) as serial_port:
        print(
            f"sending left={args.left:.3f} rad/s right={args.right:.3f} rad/s "
            f"to {args.port} @ {args.baudrate} bps, rate={args.rate:.1f} Hz"
        )
        next_tx = time.monotonic()
        while RUNNING:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            if now < next_tx:
                time.sleep(min(0.001, next_tx - now))
                continue
            serial_port.write_all(frame)
            sent += 1
            next_tx += interval

        if args.send_zero_on_exit:
            serial_port.write_all(build_vel_cmd(0.0, 0.0))
            print("sent final zero-speed command")

    print(f"sent {sent} command frames")
    return 0


def cmd_watchdog_test(args: argparse.Namespace) -> int:
    tx_frame = build_vel_cmd(args.left, args.right)
    parser = FeedbackParser()

    with SerialPort(args.port, args.baudrate) as serial_port:
        print(
            f"phase 1: send command for {args.drive_time:.2f}s "
            f"(left={args.left:.3f}, right={args.right:.3f} rad/s)"
        )
        end_drive = time.monotonic() + args.drive_time
        next_tx = time.monotonic()
        tx_interval = 1.0 / args.rate
        while RUNNING and time.monotonic() < end_drive:
            now = time.monotonic()
            if now >= next_tx:
                serial_port.write_all(tx_frame)
                next_tx += tx_interval
            chunk = serial_port.read(timeout=0.01)
            if chunk:
                parser.feed(chunk)

        print(f"phase 2: stop sending, observe for {args.observe_time:.2f}s")
        zero_seen_at: Optional[float] = None
        observe_start = time.monotonic()
        while RUNNING and time.monotonic() - observe_start < args.observe_time:
            chunk = serial_port.read(timeout=0.2)
            if not chunk:
                continue
            frames = parser.feed(chunk)
            for frame in frames:
                print(format_frame(frame))
                if (
                    abs(frame.left_delta) <= args.zero_delta_threshold
                    and abs(frame.right_delta) <= args.zero_delta_threshold
                ):
                    if zero_seen_at is None:
                        zero_seen_at = time.monotonic()

        if zero_seen_at is None:
            print("watchdog result: did not observe near-zero encoder deltas in observe window")
            return 2

        stop_delay = zero_seen_at - observe_start
        print(
            f"watchdog result: near-zero deltas first seen after {stop_delay:.3f}s "
            f"(threshold={args.zero_delta_threshold} counts)"
        )
        print("expected: approximately 0.5s plus motor/mechanical coast time")
        return 0


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 serial protocol acceptance helper")
    parser.add_argument("--port", default="/dev/ttyS7", help="serial device path")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baudrate")
    subparsers = parser.add_subparsers(dest="command", required=True)

    listen_parser = subparsers.add_parser("listen", help="listen for 0x02 feedback frames")
    listen_parser.add_argument(
        "--report-interval", type=float, default=2.0, help="summary print interval in seconds"
    )
    listen_parser.add_argument(
        "--print-every", type=int, default=10, help="print every Nth valid frame"
    )
    listen_parser.set_defaults(func=cmd_listen)

    send_parser = subparsers.add_parser("send", help="send repeated 0x01 velocity commands")
    send_parser.add_argument("--left", type=float, required=True, help="left wheel target rad/s")
    send_parser.add_argument("--right", type=float, required=True, help="right wheel target rad/s")
    send_parser.add_argument("--rate", type=float, default=50.0, help="send rate in Hz")
    send_parser.add_argument(
        "--duration", type=float, default=2.0, help="send duration in seconds, <=0 means forever"
    )
    send_parser.add_argument(
        "--send-zero-on-exit",
        action="store_true",
        help="send one final zero-speed frame before exit",
    )
    send_parser.set_defaults(func=cmd_send)

    watchdog_parser = subparsers.add_parser(
        "watchdog-test", help="send for a while, then stop and observe encoder deltas"
    )
    watchdog_parser.add_argument("--left", type=float, required=True, help="left wheel target rad/s")
    watchdog_parser.add_argument("--right", type=float, required=True, help="right wheel target rad/s")
    watchdog_parser.add_argument("--rate", type=float, default=50.0, help="command send rate in Hz")
    watchdog_parser.add_argument(
        "--drive-time", type=float, default=1.5, help="active command phase in seconds"
    )
    watchdog_parser.add_argument(
        "--observe-time", type=float, default=2.0, help="observe phase duration in seconds"
    )
    watchdog_parser.add_argument(
        "--zero-delta-threshold",
        type=int,
        default=5,
        help="consider encoder stationary when abs(delta) <= threshold",
    )
    watchdog_parser.set_defaults(func=cmd_watchdog_test)

    return parser


def main() -> int:
    signal.signal(signal.SIGINT, handle_sigint)
    args = build_argparser().parse_args()
    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:  # pragma: no cover - CLI path
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
