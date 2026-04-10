#!/usr/bin/env python3
import argparse
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class LeftRadiusCalibrator(Node):
    def __init__(
        self,
        odom_topic: str,
        cmd_topic: str,
        linear_speed: float,
        target_distance: float,
        wheel_separation: float,
        current_left_multiplier: float,
        settle_time: float,
    ):
        super().__init__("left_radius_calibrator")
        self.linear_speed = linear_speed
        self.target_distance = target_distance
        self.wheel_separation = wheel_separation
        self.current_left_multiplier = current_left_multiplier
        self.settle_time = settle_time

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.timer = self.create_timer(0.02, self.control_cb)

        self.started = False
        self.finished = False
        self.start_time: Optional[float] = None
        self.stop_time: Optional[float] = None

        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.path_length = 0.0
        self.samples = 0

        self.get_logger().info(
            f"Waiting for odom on {odom_topic}. Will publish straight cmd to {cmd_topic}."
        )

    def odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.prev_x = x
            self.prev_y = y
            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw
            self.get_logger().info("First odom frame received. Starting settle timer.")
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        self.path_length += math.hypot(dx, dy)
        self.prev_x = x
        self.prev_y = y
        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.samples += 1

    def control_cb(self) -> None:
        if self.start_x is None or self.finished:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if not self.started:
            if self.start_time is None:
                self.start_time = now
            if now - self.start_time >= self.settle_time:
                self.started = True
                self.get_logger().info(
                    f"Starting straight run: v={self.linear_speed:.3f} m/s, target={self.target_distance:.3f} m"
                )
            else:
                self.publish_cmd(0.0)
                return

        if self.path_length < self.target_distance:
            self.publish_cmd(self.linear_speed)
            return

        self.publish_cmd(0.0)
        if self.stop_time is None:
            self.stop_time = now
            self.get_logger().info("Target distance reached. Settling before summary.")
            return

        if now - self.stop_time >= self.settle_time:
            self.finished = True
            self.print_summary()
            self.destroy_node()
            rclpy.shutdown()

    def publish_cmd(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def print_summary(self) -> None:
        dx = self.last_x - self.start_x
        dy = self.last_y - self.start_y
        yaw_err = wrap_angle(self.last_yaw - self.start_yaw)

        forward = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        lateral = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        distance_used = max(self.path_length, 1e-6)

        left_distance = distance_used - self.wheel_separation * yaw_err / 2.0
        right_distance = distance_used + self.wheel_separation * yaw_err / 2.0
        recommended_left = self.current_left_multiplier * (left_distance / right_distance)

        direction = "left"
        if yaw_err < 0.0:
            direction = "right"

        print("\n=== Left Wheel Radius Calibration Summary ===")
        print(f"odom samples                 : {self.samples}")
        print(f"path length                  : {self.path_length:.6f} m")
        print(f"forward displacement         : {forward:.6f} m")
        print(f"lateral displacement         : {lateral:.6f} m")
        print(f"final yaw error              : {yaw_err:.6f} rad ({math.degrees(yaw_err):.3f} deg)")
        print(f"drift direction              : {direction if abs(yaw_err) > 1e-6 else 'straight'}")
        print("")
        print("Wheel-distance estimate from odom")
        print(f"left wheel distance          : {left_distance:.6f} m")
        print(f"right wheel distance         : {right_distance:.6f} m")
        print("")
        print("Multiplier suggestion")
        print(f"current left multiplier      : {self.current_left_multiplier:.6f}")
        print(f"recommended left multiplier  : {recommended_left:.6f}")
        print(f"delta                        : {recommended_left - self.current_left_multiplier:+.6f}")
        print("")
        print("Interpretation")
        print("- If robot drifted right, the suggested multiplier should usually be larger.")
        print("- Re-run this script after updating hw_controllers.yaml and restarting bringup.")
        print("- Use /odom for calibration. /odometry/filtered may hide wheel mismatch with IMU correction.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drive straight and estimate a better left_wheel_radius_multiplier from /odom drift."
    )
    parser.add_argument("--odom-topic", default="/odom", help="Odometry topic to use. Default: /odom")
    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel_keyboard",
        help="Twist topic to publish straight command to. Default: /cmd_vel_keyboard",
    )
    parser.add_argument("--speed", type=float, default=0.4, help="Straight-line speed in m/s")
    parser.add_argument("--distance", type=float, default=2.0, help="Target path length in meters")
    parser.add_argument("--wheel-separation", type=float, required=True, help="Wheel separation in meters")
    parser.add_argument(
        "--current-left-multiplier",
        type=float,
        required=True,
        help="Current left_wheel_radius_multiplier from hw_controllers.yaml",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=1.0,
        help="Seconds to wait before and after the run",
    )
    args = parser.parse_args()

    rclpy.init()
    node = LeftRadiusCalibrator(
        odom_topic=args.odom_topic,
        cmd_topic=args.cmd_topic,
        linear_speed=args.speed,
        target_distance=args.distance,
        wheel_separation=args.wheel_separation,
        current_left_multiplier=args.current_left_multiplier,
        settle_time=args.settle_time,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
