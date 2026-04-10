#!/usr/bin/env python3
import argparse
import math
import os
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu


Vector3 = Tuple[float, float, float]


def wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def quaternion_to_rpy(msg: Imu) -> Vector3:
    q = msg.orientation

    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def covariance_1d(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return sum((value - avg) ** 2 for value in values) / (len(values) - 1)


def covariance_3x3(samples: Sequence[Vector3]) -> List[List[float]]:
    if len(samples) < 2:
        return [[0.0, 0.0, 0.0] for _ in range(3)]

    means = [mean([sample[i] for sample in samples]) for i in range(3)]
    matrix = [[0.0, 0.0, 0.0] for _ in range(3)]

    for row in range(3):
        for col in range(3):
            accum = 0.0
            for sample in samples:
                accum += (sample[row] - means[row]) * (sample[col] - means[col])
            matrix[row][col] = accum / (len(samples) - 1)

    return matrix


def flatten_matrix_row_major(matrix: Sequence[Sequence[float]]) -> List[float]:
    return [matrix[row][col] for row in range(3) for col in range(3)]


def fmt_matrix(matrix: Sequence[Sequence[float]]) -> str:
    rows = []
    for row in matrix:
        rows.append("[" + ", ".join(f"{value:.9e}" for value in row) + "]")
    return "\n".join(rows)


def fmt_flat(values: Sequence[float]) -> str:
    return "[" + ", ".join(f"{value:.9e}" for value in values) + "]"


class MeasurementBuffer:
    def __init__(self) -> None:
        self.raw_accel: List[Vector3] = []
        self.raw_gyro: List[Vector3] = []
        self.fused_rpy: List[Vector3] = []

        self._last_fused_yaw: Optional[float] = None
        self._unwrapped_fused_yaw: Optional[float] = None

    def add_raw_imu(self, msg: Imu) -> None:
        self.raw_accel.append((
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        ))
        self.raw_gyro.append((
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        ))

    def add_fused_imu(self, msg: Imu) -> None:
        roll, pitch, yaw = quaternion_to_rpy(msg)

        if self._last_fused_yaw is None:
            self._last_fused_yaw = yaw
            self._unwrapped_fused_yaw = yaw
        else:
            self._unwrapped_fused_yaw += wrap_angle(yaw - self._last_fused_yaw)
            self._last_fused_yaw = yaw

        self.fused_rpy.append((roll, pitch, self._unwrapped_fused_yaw))


class LiveCollector(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("imu_covariance_probe")
        self.args = args
        self.buffer = MeasurementBuffer()
        self.start_time_ns: Optional[int] = None
        self.done = False

        self.raw_sub = self.create_subscription(Imu, args.raw_topic, self.raw_cb, 100)
        self.fused_sub = self.create_subscription(Imu, args.fused_topic, self.fused_cb, 100)
        self.timer = self.create_timer(0.5, self.progress_cb)

        self.get_logger().info(
            f"Listening live for {args.duration:.1f}s: "
            f"raw={args.raw_topic}, fused={args.fused_topic}. Keep robot still."
        )

    def _stamp_ns(self, msg: Imu) -> int:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        return stamp_ns

    def _within_window(self, msg: Imu) -> bool:
        stamp_ns = self._stamp_ns(msg)
        if self.start_time_ns is None:
            self.start_time_ns = stamp_ns
        elapsed_s = (stamp_ns - self.start_time_ns) / 1e9
        if elapsed_s > self.args.duration:
            self.finish()
            return False
        return True

    def raw_cb(self, msg: Imu) -> None:
        if self.done or not self._within_window(msg):
            return
        self.buffer.add_raw_imu(msg)

    def fused_cb(self, msg: Imu) -> None:
        if self.done or not self._within_window(msg):
            return
        self.buffer.add_fused_imu(msg)

    def progress_cb(self) -> None:
        if self.done:
            return

        if self.start_time_ns is None:
            self.get_logger().info("Waiting for IMU messages...")
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self.start_time_ns) / 1e9
        self.get_logger().info(
            f"elapsed={elapsed_s:5.1f}s raw_samples={len(self.buffer.raw_gyro):4d} "
            f"fused_samples={len(self.buffer.fused_rpy):4d}"
        )
        if elapsed_s >= self.args.duration:
            self.finish()

    def finish(self) -> None:
        if self.done:
            return
        self.done = True
        print_report(self.buffer, self.args)
        self.destroy_node()
        rclpy.shutdown()


def collect_from_bag(args: argparse.Namespace) -> MeasurementBuffer:
    if not args.bag:
        raise ValueError("--bag is required for bag mode")

    bag_path = args.bag
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=args.storage_id),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted_topics = {args.raw_topic, args.fused_topic}
    missing_topics = [topic for topic in wanted_topics if topic not in topic_types]
    if missing_topics:
        raise RuntimeError(f"Missing topics in bag: {', '.join(missing_topics)}")

    topic_classes = {topic: get_message(topic_types[topic]) for topic in wanted_topics}

    buffer = MeasurementBuffer()
    first_time_ns: Optional[int] = None
    start_ns: Optional[int] = None
    end_ns: Optional[int] = None

    while reader.has_next():
        topic, raw_data, record_time_ns = reader.read_next()
        if topic not in wanted_topics:
            continue

        if first_time_ns is None:
            first_time_ns = record_time_ns
            start_ns = first_time_ns + int(args.start_offset * 1e9)
            if args.duration > 0.0:
                end_ns = start_ns + int(args.duration * 1e9)

        if record_time_ns < start_ns:
            continue
        if end_ns is not None and record_time_ns > end_ns:
            break

        msg = deserialize_message(raw_data, topic_classes[topic])
        if topic == args.raw_topic:
            buffer.add_raw_imu(msg)
        elif topic == args.fused_topic:
            buffer.add_fused_imu(msg)

    return buffer


def print_report(buffer: MeasurementBuffer, args: argparse.Namespace) -> None:
    accel_cov = covariance_3x3(buffer.raw_accel)
    gyro_cov = covariance_3x3(buffer.raw_gyro)
    orient_cov = covariance_3x3(buffer.fused_rpy)

    yaw_floor_rad = math.radians(args.yaw_std_floor_deg)
    yaw_floor_var = yaw_floor_rad * yaw_floor_rad
    orient_cov_seed = [row[:] for row in orient_cov]
    orient_cov_seed[2][2] = max(orient_cov_seed[2][2], yaw_floor_var)

    print("\n=== IMU covariance probe ===")
    print(f"raw samples              : {len(buffer.raw_gyro)}")
    print(f"fused samples            : {len(buffer.fused_rpy)}")
    print("")

    if not buffer.raw_gyro:
        print("No raw IMU samples found on the selected topic.")
    else:
        print("linear_acceleration_covariance (measured)")
        print(fmt_matrix(accel_cov))
        print("")
        print("angular_velocity_covariance (measured)")
        print(fmt_matrix(gyro_cov))
        print("")
        print("Paste into sensor_msgs/Imu for /imu/data_raw")
        print(f"linear_acceleration_covariance = {fmt_flat(flatten_matrix_row_major(accel_cov))}")
        print(f"angular_velocity_covariance    = {fmt_flat(flatten_matrix_row_major(gyro_cov))}")
        print("orientation_covariance         = [-1.000000000e+00, 0, 0, 0, 0, 0, 0, 0, 0]")
        print("")

    if not buffer.fused_rpy:
        print("No fused IMU samples found on the selected topic.")
    else:
        print("orientation_covariance (measured roll/pitch/yaw)")
        print(fmt_matrix(orient_cov))
        print("")
        print(
            "orientation_covariance (EKF seed, conservative yaw floor "
            f"{args.yaw_std_floor_deg:.1f} deg)"
        )
        print(fmt_matrix(orient_cov_seed))
        print("")
        print("Paste into fused /imu/data publisher if you set covariance there")
        print(f"orientation_covariance_measured = {fmt_flat(flatten_matrix_row_major(orient_cov))}")
        print(f"orientation_covariance_seed     = {fmt_flat(flatten_matrix_row_major(orient_cov_seed))}")
        print("")

    print("Notes")
    print("- Record the robot at rest on level ground for 60-180 s.")
    print("- /imu/data_raw covariance should come from raw accel/gyro, not from fused orientation.")
    print("- In 6-axis mode (use_mag: false), do not set yaw covariance too small.")
    print("- The seed is only a starting point; tune EKF trust after straight-run and in-place-turn tests.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute IMU covariance from live topics or from a rosbag. "
            "Use stationary data for 60-180 s."
        )
    )
    parser.add_argument("--bag", type=str, default="", help="Path to rosbag2 directory. Leave empty for live mode.")
    parser.add_argument("--storage-id", type=str, default="sqlite3", help="rosbag2 storage id, default: sqlite3.")
    parser.add_argument("--raw-topic", type=str, default="/imu/data_raw", help="Raw IMU topic.")
    parser.add_argument("--fused-topic", type=str, default="/imu/data", help="Fused IMU topic.")
    parser.add_argument("--start-offset", type=float, default=0.0, help="Skip this many seconds from the bag start.")
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Capture duration in seconds. In bag mode, <= 0 means use all remaining data.",
    )
    parser.add_argument(
        "--yaw-std-floor-deg",
        type=float,
        default=5.0,
        help="Minimum yaw standard deviation for EKF seed in 6-axis mode.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.bag:
        buffer = collect_from_bag(args)
        print_report(buffer, args)
        return

    rclpy.init()
    node = LiveCollector(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
