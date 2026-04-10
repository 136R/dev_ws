#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


WheelPair = Tuple[float, float]


@dataclass
class Sample:
    t: float
    left_vel: float
    right_vel: float
    left_target: float
    right_target: float


@dataclass
class AlphaStats:
    alpha: float
    left_stationary_rms: float
    right_stationary_rms: float
    left_moving_rms_err: float
    right_moving_rms_err: float
    left_step_delay_s: float
    right_step_delay_s: float
    score: float


def rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def ema(values: Sequence[float], alpha: float) -> List[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        out.append(alpha * out[-1] + (1.0 - alpha) * value)
    return out


def twist_to_wheels(linear_x: float, angular_z: float, wheel_sep: float, wheel_radius: float) -> WheelPair:
    left = (linear_x - angular_z * wheel_sep / 2.0) / wheel_radius
    right = (linear_x + angular_z * wheel_sep / 2.0) / wheel_radius
    return left, right


def find_joint_index(msg: JointState, joint_name: str) -> Optional[int]:
    try:
        return msg.name.index(joint_name)
    except ValueError:
        return None


class AlphaProbe(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("odom_lpf_alpha_probe")
        self.args = args

        self.samples: List[Sample] = []
        self._start_time: Optional[float] = None
        self._last_target: WheelPair = (0.0, 0.0)
        self._left_index: Optional[int] = None
        self._right_index: Optional[int] = None
        self._warned_missing_joint = False
        self._warned_missing_velocity = False
        self._phase_index = -1
        self._phase_start_time: Optional[float] = None
        self._phases = [
            ("static_1", 0.0, 0.0, args.static_duration_1),
            ("forward_0p2", args.speed_1, 0.0, args.run_duration_1),
            ("static_2", 0.0, 0.0, args.static_duration_2),
            ("forward_0p3", args.speed_2, 0.0, args.run_duration_2),
            ("static_3", 0.0, 0.0, args.static_duration_3),
        ]

        self.create_subscription(JointState, args.joint_states_topic, self.joint_cb, 100)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.timer = self.create_timer(0.05, self.control_cb)
        self.progress_timer = self.create_timer(0.5, self.progress_cb)

        self.get_logger().info(
            "Recording wheel velocity and auto-driving the LPF probe sequence."
        )
        self.get_logger().info(
            f"joint_states={args.joint_states_topic}, cmd_topic={args.cmd_topic}, "
            f"wheel_radius={args.wheel_radius:.4f} m"
        )

    def joint_cb(self, msg: JointState) -> None:
        if self._left_index is None or self._right_index is None:
            self._left_index = find_joint_index(msg, self.args.left_joint)
            self._right_index = find_joint_index(msg, self.args.right_joint)
            if self._left_index is None or self._right_index is None:
                if not self._warned_missing_joint:
                    self.get_logger().warn(
                        f"Joint names not found yet. left={self.args.left_joint}, right={self.args.right_joint}"
                    )
                    self._warned_missing_joint = True
                return

        if len(msg.velocity) <= max(self._left_index, self._right_index):
            if not self._warned_missing_velocity:
                self.get_logger().warn("joint_states has no velocity entries yet")
                self._warned_missing_velocity = True
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        t = stamp_ns / 1e9

        if self._start_time is None:
            self._start_time = t

        elapsed = t - self._start_time
        if elapsed > self.args.duration:
            self.finish()
            return

        self.samples.append(
            Sample(
                t=elapsed,
                left_vel=float(msg.velocity[self._left_index]),
                right_vel=float(msg.velocity[self._right_index]),
                left_target=self._last_target[0],
                right_target=self._last_target[1],
            )
        )

    def control_cb(self) -> None:
        if self._start_time is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._start_time

        if self._phase_start_time is None:
            self._phase_index = 0
            self._phase_start_time = elapsed
            self.log_phase()

        phase_name, linear_x, angular_z, duration = self._phases[self._phase_index]
        phase_elapsed = elapsed - self._phase_start_time

        if phase_elapsed >= duration:
            self._phase_index += 1
            if self._phase_index >= len(self._phases):
                self.publish_cmd(0.0, 0.0)
                self.finish()
                return
            self._phase_start_time = elapsed
            self.log_phase()
            phase_name, linear_x, angular_z, duration = self._phases[self._phase_index]

        self.publish_cmd(linear_x, angular_z)
        self._last_target = twist_to_wheels(
            linear_x,
            angular_z,
            self.args.wheel_separation,
            self.args.wheel_radius,
        )

    def log_phase(self) -> None:
        phase_name, linear_x, angular_z, duration = self._phases[self._phase_index]
        self.get_logger().info(
            f"Phase {self._phase_index + 1}/{len(self._phases)}: {phase_name}, "
            f"cmd=({linear_x:.3f} m/s, {angular_z:.3f} rad/s), duration={duration:.1f}s"
        )

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def progress_cb(self) -> None:
        if self._start_time is None:
            self.get_logger().info("Waiting for joint_states...")
            return
        if self.samples:
            self.get_logger().info(
                f"elapsed={self.samples[-1].t:5.1f}s samples={len(self.samples):4d} "
                f"targetL={self.samples[-1].left_target:+.2f} targetR={self.samples[-1].right_target:+.2f}"
            )

    def finish(self) -> None:
        if not self.samples:
            self.get_logger().error("No samples captured.")
            self.destroy_node()
            rclpy.shutdown()
            return

        results = evaluate_alphas(
            self.samples,
            stationary_cmd_threshold=self.args.stationary_cmd_threshold,
            moving_cmd_threshold=self.args.moving_cmd_threshold,
            step_threshold=self.args.step_threshold,
        )
        print_report(results)
        self.publish_cmd(0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown()


def evaluate_step_delay(
    times: Sequence[float],
    targets: Sequence[float],
    filtered: Sequence[float],
    step_threshold: float,
) -> float:
    steps: List[float] = []
    for i in range(1, len(times)):
        prev = targets[i - 1]
        curr = targets[i]
        if abs(curr - prev) < step_threshold:
            continue
        if abs(curr) < step_threshold:
            continue

        target_abs = abs(curr)
        reach = 0.9 * target_abs
        trigger_t = times[i]
        reached_t: Optional[float] = None

        for j in range(i, len(times)):
            if abs(filtered[j]) >= reach:
                reached_t = times[j]
                break

        if reached_t is not None:
            steps.append(max(0.0, reached_t - trigger_t))

    return sum(steps) / len(steps) if steps else 0.0


def evaluate_alphas(
    samples: Sequence[Sample],
    stationary_cmd_threshold: float,
    moving_cmd_threshold: float,
    step_threshold: float,
) -> List[AlphaStats]:
    times = [s.t for s in samples]
    left_vel = [s.left_vel for s in samples]
    right_vel = [s.right_vel for s in samples]
    left_target = [s.left_target for s in samples]
    right_target = [s.right_target for s in samples]

    stationary_left = [v for v, tgt in zip(left_vel, left_target) if abs(tgt) <= stationary_cmd_threshold]
    stationary_right = [v for v, tgt in zip(right_vel, right_target) if abs(tgt) <= stationary_cmd_threshold]

    results: List[AlphaStats] = []
    for idx in range(0, 20):
        alpha = idx * 0.05
        filtered_left = ema(left_vel, alpha)
        filtered_right = ema(right_vel, alpha)

        left_stationary_filtered = [
            v for v, tgt in zip(filtered_left, left_target) if abs(tgt) <= stationary_cmd_threshold
        ]
        right_stationary_filtered = [
            v for v, tgt in zip(filtered_right, right_target) if abs(tgt) <= stationary_cmd_threshold
        ]
        left_moving_error = [
            v - tgt for v, tgt in zip(filtered_left, left_target) if abs(tgt) >= moving_cmd_threshold
        ]
        right_moving_error = [
            v - tgt for v, tgt in zip(filtered_right, right_target) if abs(tgt) >= moving_cmd_threshold
        ]

        left_delay = evaluate_step_delay(times, left_target, filtered_left, step_threshold)
        right_delay = evaluate_step_delay(times, right_target, filtered_right, step_threshold)

        left_stationary_rms = rms(left_stationary_filtered) if left_stationary_filtered else rms(stationary_left)
        right_stationary_rms = rms(right_stationary_filtered) if right_stationary_filtered else rms(stationary_right)
        left_moving_rms_err = rms(left_moving_error)
        right_moving_rms_err = rms(right_moving_error)

        score = (
            2.0 * left_stationary_rms +
            2.0 * right_stationary_rms +
            0.35 * left_moving_rms_err +
            0.35 * right_moving_rms_err +
            4.0 * left_delay +
            4.0 * right_delay
        )

        results.append(
            AlphaStats(
                alpha=alpha,
                left_stationary_rms=left_stationary_rms,
                right_stationary_rms=right_stationary_rms,
                left_moving_rms_err=left_moving_rms_err,
                right_moving_rms_err=right_moving_rms_err,
                left_step_delay_s=left_delay,
                right_step_delay_s=right_delay,
                score=score,
            )
        )

    return results


def print_report(results: Sequence[AlphaStats]) -> None:
    best_left = min(results, key=lambda item: (item.left_stationary_rms, item.left_step_delay_s + 0.2 * item.left_moving_rms_err))
    best_right = min(results, key=lambda item: (item.right_stationary_rms, item.right_step_delay_s + 0.2 * item.right_moving_rms_err))
    best_overall = min(results, key=lambda item: item.score)

    print("\n=== ODOM LPF Alpha Probe ===")
    print("alpha  left_noise  right_noise  left_err  right_err  left_delay  right_delay  score")
    for item in results:
        print(
            f"{item.alpha:>4.2f}  "
            f"{item.left_stationary_rms:>10.4f}  "
            f"{item.right_stationary_rms:>11.4f}  "
            f"{item.left_moving_rms_err:>8.4f}  "
            f"{item.right_moving_rms_err:>9.4f}  "
            f"{item.left_step_delay_s:>10.3f}  "
            f"{item.right_step_delay_s:>11.3f}  "
            f"{item.score:>6.3f}"
        )

    print("\nRecommendation")
    print(f"- Best overall alpha          : {best_overall.alpha:.2f}")
    print(f"- Best left-wheel alpha       : {best_left.alpha:.2f}")
    print(f"- Best right-wheel alpha      : {best_right.alpha:.2f}")

    print("\nHow to apply")
    print(f"- Set ODOM_LPF_ALPHA_LEFT  to about {best_left.alpha:.2f}")
    print(f"- Set ODOM_LPF_ALPHA_RIGHT to about {best_right.alpha:.2f}")
    print("- If you prefer one shared value, start from the best overall alpha.")
    print("- If launch/stop feels too soft on the real robot, decrease alpha by 0.05.")
    print("- If joint velocity is still visibly noisy while driving straight, increase alpha by 0.05.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Auto-drive a 0.2/0.3 m/s sequence and probe a suitable encoder LPF alpha from /joint_states."
    )
    parser.add_argument("--joint-states-topic", default="/joint_states", help="JointState topic.")
    parser.add_argument("--cmd-topic", default="/diff_cont/cmd_vel_unstamped", help="Twist command topic.")
    parser.add_argument("--left-joint", default="left_wheel_joint", help="Left wheel joint name.")
    parser.add_argument("--right-joint", default="right_wheel_joint", help="Right wheel joint name.")
    parser.add_argument("--wheel-separation", type=float, default=0.193, help="Wheel separation in meters.")
    parser.add_argument("--wheel-radius", type=float, default=0.035, help="Wheel radius in meters.")
    parser.add_argument("--speed-1", type=float, default=0.2, help="First forward test speed in m/s.")
    parser.add_argument("--speed-2", type=float, default=0.3, help="Second forward test speed in m/s.")
    parser.add_argument("--static-duration-1", type=float, default=3.0, help="Initial static duration in seconds.")
    parser.add_argument("--run-duration-1", type=float, default=4.0, help="0.2 m/s phase duration in seconds.")
    parser.add_argument("--static-duration-2", type=float, default=2.0, help="Middle static duration in seconds.")
    parser.add_argument("--run-duration-2", type=float, default=4.0, help="0.3 m/s phase duration in seconds.")
    parser.add_argument("--static-duration-3", type=float, default=3.0, help="Final static duration in seconds.")
    parser.add_argument("--stationary-cmd-threshold", type=float, default=0.15, help="Wheel target rad/s threshold for stationary samples.")
    parser.add_argument("--moving-cmd-threshold", type=float, default=4.0, help="Wheel target rad/s threshold for moving samples.")
    parser.add_argument("--step-threshold", type=float, default=1.5, help="Minimum target step size in rad/s.")
    args = parser.parse_args()
    args.duration = (
        args.static_duration_1 +
        args.run_duration_1 +
        args.static_duration_2 +
        args.run_duration_2 +
        args.static_duration_3
    )

    rclpy.init()
    node = AlphaProbe(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
