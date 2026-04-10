#!/usr/bin/env python3
import argparse
import math
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def variance(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return sum((value - avg) ** 2 for value in values) / (len(values) - 1)


def stddev(values: List[float]) -> float:
    return math.sqrt(max(variance(values), 0.0))


class OdomVxProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("odom_vx_covariance_probe")
        self.args = args
        self.msg_start_time_ns: Optional[int] = None
        self.wall_start_time_ns: Optional[int] = None
        self.done = False

        self.vx_samples: List[float] = []
        self.error_samples: List[float] = []
        self.forward_vx_samples: List[float] = []
        self.reverse_vx_samples: List[float] = []
        self.forward_error_samples: List[float] = []
        self.reverse_error_samples: List[float] = []
        self.dropped_warmup = 0
        self.dropped_deadband = 0

        self.sub = self.create_subscription(Odometry, args.topic, self.odom_cb, 100)
        self.timer = self.create_timer(0.5, self.progress_cb)

        if args.mode == "stationary":
            mode_desc = "robot must stay still"
        elif args.dynamic_profile == "single_direction":
            mode_desc = f"robot should drive straight at {args.reference_speed:.3f} m/s"
        else:
            mode_desc = (
                "robot may move forward and reverse; samples near zero speed "
                f"are ignored with deadband={args.motion_deadband:.3f} m/s"
            )
        self.get_logger().info(
            f"Listening live on {args.topic} for {args.duration:.1f}s, "
            f"warmup={args.warmup:.1f}s, mode={args.mode}. {mode_desc}."
        )

    def stamp_ns(self, msg: Odometry) -> int:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        return stamp_ns

    def odom_cb(self, msg: Odometry) -> None:
        if self.done:
            return

        stamp_ns = self.stamp_ns(msg)
        if self.msg_start_time_ns is None:
            self.msg_start_time_ns = stamp_ns
            self.wall_start_time_ns = self.get_clock().now().nanoseconds

        elapsed = (stamp_ns - self.msg_start_time_ns) / 1e9
        if elapsed > self.args.duration:
            self.finish()
            return

        vx = float(msg.twist.twist.linear.x)
        if elapsed < self.args.warmup:
            self.dropped_warmup += 1
            return

        if self.args.mode == "dynamic":
            if self.args.dynamic_profile == "single_direction":
                self.vx_samples.append(vx)
                self.error_samples.append(vx - self.args.reference_speed)
            else:
                if abs(vx) < self.args.motion_deadband:
                    self.dropped_deadband += 1
                    return

                self.vx_samples.append(vx)
                if vx > 0.0:
                    self.forward_vx_samples.append(vx)
                    err = vx - abs(self.args.reference_speed)
                    self.forward_error_samples.append(err)
                    self.error_samples.append(err)
                else:
                    self.reverse_vx_samples.append(vx)
                    err = vx + abs(self.args.reference_speed)
                    self.reverse_error_samples.append(err)
                    self.error_samples.append(err)
        else:
            self.vx_samples.append(vx)

    def progress_cb(self) -> None:
        if self.done:
            return

        if self.wall_start_time_ns is None:
            self.get_logger().info("Waiting for /odom messages...")
            return

        elapsed = (self.get_clock().now().nanoseconds - self.wall_start_time_ns) / 1e9
        if self.args.mode == "dynamic" and self.args.dynamic_profile == "round_trip":
            self.get_logger().info(
                f"elapsed={elapsed:5.1f}s kept={len(self.vx_samples):4d} "
                f"fwd={len(self.forward_vx_samples):4d} rev={len(self.reverse_vx_samples):4d} "
                f"warmup_dropped={self.dropped_warmup:4d} deadband_dropped={self.dropped_deadband:4d}"
            )
        else:
            self.get_logger().info(
                f"elapsed={elapsed:5.1f}s kept={len(self.vx_samples):4d} "
                f"warmup_dropped={self.dropped_warmup:4d}"
            )
        if elapsed >= self.args.duration:
            self.finish()

    def finish(self) -> None:
        if self.done:
            return
        self.done = True
        print_report(
            self.args,
            self.vx_samples,
            self.error_samples,
            self.forward_vx_samples,
            self.reverse_vx_samples,
            self.forward_error_samples,
            self.reverse_error_samples,
            self.dropped_deadband,
        )
        self.destroy_node()
        rclpy.shutdown()


def print_report(
    args: argparse.Namespace,
    vx_samples: List[float],
    error_samples: List[float],
    forward_vx_samples: List[float],
    reverse_vx_samples: List[float],
    forward_error_samples: List[float],
    reverse_error_samples: List[float],
    dropped_deadband: int,
) -> None:
    vx_mean = mean(vx_samples)
    vx_var = variance(vx_samples)
    vx_std = stddev(vx_samples)

    print("\n=== Odom vx covariance probe ===")
    print(f"mode                     : {args.mode}")
    print(f"topic                    : {args.topic}")
    print(f"capture duration         : {args.duration:.1f} s")
    print(f"warmup discarded         : {args.warmup:.1f} s")
    print(f"kept samples             : {len(vx_samples)}")
    if args.mode == "dynamic":
        print(f"dynamic profile          : {args.dynamic_profile}")
        if args.dynamic_profile == "round_trip":
            print(f"motion deadband          : {args.motion_deadband:.3f} m/s")
            print(f"deadband dropped         : {dropped_deadband}")
    print("")

    if not vx_samples:
        print("No odom samples captured after warmup.")
        return

    print(f"vx mean                  : {vx_mean:.9e} m/s")
    print(f"vx stddev                : {vx_std:.9e} m/s")
    print(f"vx variance              : {vx_var:.9e} (m/s)^2")
    print("")

    if args.mode == "stationary":
        print("Suggested use")
        print(f"- Lower-bound vx covariance: {vx_var:.9e}")
        print("- Use this as the minimum noise floor only.")
        print("- Final EKF /odom vx covariance is usually larger than this.")
    else:
        err_mean = mean(error_samples)
        err_var = variance(error_samples)
        err_std = stddev(error_samples)

        print(f"reference speed          : {args.reference_speed:.9e} m/s")
        print(f"vx error mean            : {err_mean:.9e} m/s")
        print(f"vx error stddev          : {err_std:.9e} m/s")
        print(f"vx error variance        : {err_var:.9e} (m/s)^2")
        print("")

        if args.dynamic_profile == "round_trip":
            print(f"forward kept             : {len(forward_vx_samples)}")
            print(f"reverse kept             : {len(reverse_vx_samples)}")
            if forward_vx_samples:
                print(f"forward vx mean          : {mean(forward_vx_samples):.9e} m/s")
                print(f"forward err variance     : {variance(forward_error_samples):.9e} (m/s)^2")
            if reverse_vx_samples:
                print(f"reverse vx mean          : {mean(reverse_vx_samples):.9e} m/s")
                print(f"reverse err variance     : {variance(reverse_error_samples):.9e} (m/s)^2")
            print("")

        print("Suggested use")
        print(f"- Candidate /odom vx covariance: {err_var:.9e}")
        if args.dynamic_profile == "single_direction":
            print("- Run this multiple times on your normal floor and use the larger stable result.")
            print("- If starts/stops contaminate the result, increase --warmup and keep speed constant.")
        else:
            print("- Forward and reverse segments are scored against +reference-speed and -reference-speed separately.")
            print("- Samples near zero speed are ignored so stop-and-turn phases do not bias the result.")
            print("- If too many samples are dropped, reduce --motion-deadband slightly.")

    print("")
    print("Example")
    print(
        "twist_covariance_diagonal: "
        f"[{(variance(error_samples) if args.mode == 'dynamic' else vx_var):.9e}, 1e6, 1e6, 1e6, 1e6, 1e6]"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Probe /odom linear.x covariance in live mode. "
            "Use stationary mode for noise floor, dynamic mode for speed-tracking error."
        )
    )
    parser.add_argument(
        "--mode",
        choices=["stationary", "dynamic"],
        default="stationary",
        help="stationary: robot still; dynamic: drive straight at a constant speed.",
    )
    parser.add_argument("--topic", type=str, default="/odom", help="Odometry topic.")
    parser.add_argument("--duration", type=float, default=60.0, help="Capture duration in seconds.")
    parser.add_argument(
        "--warmup",
        type=float,
        default=3.0,
        help="Discard initial seconds to avoid startup and acceleration transients.",
    )
    parser.add_argument(
        "--reference-speed",
        type=float,
        default=0.2,
        help="Expected true linear speed in dynamic mode, meters per second.",
    )
    parser.add_argument(
        "--dynamic-profile",
        choices=["single_direction", "round_trip"],
        default="single_direction",
        help=(
            "single_direction: whole run uses one reference speed. "
            "round_trip: treat positive and negative vx as separate forward/reverse segments."
        ),
    )
    parser.add_argument(
        "--motion-deadband",
        type=float,
        default=0.03,
        help="Ignore |vx| below this threshold in round_trip mode, meters per second.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    rclpy.init()
    node = OdomVxProbe(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
