#!/usr/bin/env python3
# Copyright 2024 my_bot_hw
# SPDX-License-Identifier: Apache-2.0

import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class TopicStats:
    def __init__(self, topic_name: str, expected_hz: float, kind: str):
        self.topic_name = topic_name
        self.expected_hz = expected_hz
        self.kind = kind
        self.count = 0
        self.last_receive_ns = None
        self.last_stamp_ns = None
        self.last_interval_ms = None
        self.max_interval_ms = 0.0
        self.intervals_ms = deque(maxlen=200)
        self.last_wz = None
        self.last_yaw_deg = None
        self.last_vx = None
        self.over_20ms_count = 0
        self.over_30ms_count = 0
        self.over_50ms_count = 0
        self.stale_count = 0
        self.no_msg_count = 0
        self.current_stale_streak = 0
        self.max_stale_streak = 0

    def update_common(self, now_ns: int, stamp_ns: int):
        if self.last_receive_ns is not None:
            interval_ms = (now_ns - self.last_receive_ns) / 1e6
            self.last_interval_ms = interval_ms
            self.max_interval_ms = max(self.max_interval_ms, interval_ms)
            self.intervals_ms.append(interval_ms)
            if interval_ms > 20.0:
                self.over_20ms_count += 1
            if interval_ms > 30.0:
                self.over_30ms_count += 1
            if interval_ms > 50.0:
                self.over_50ms_count += 1
        self.last_receive_ns = now_ns
        self.last_stamp_ns = stamp_ns if stamp_ns > 0 else None
        self.count += 1

    def avg_hz(self):
        if not self.intervals_ms:
            return 0.0
        avg_ms = sum(self.intervals_ms) / len(self.intervals_ms)
        return 1000.0 / avg_ms if avg_ms > 1e-6 else 0.0

    def receive_age_ms(self, now_ns: int):
        if self.last_receive_ns is None:
            return None
        return (now_ns - self.last_receive_ns) / 1e6

    def stamp_age_ms(self, now_ns: int):
        if self.last_stamp_ns is None:
            return None
        return (now_ns - self.last_stamp_ns) / 1e6

    def gap_status(self, now_ns: int):
        age_ms = self.receive_age_ms(now_ns)
        if age_ms is None:
            return 'NO_MSG'
        warn_gap_ms = (1000.0 / self.expected_hz) * 2.5
        if age_ms > warn_gap_ms:
            return 'STALE'
        return 'OK'

    def update_status_counters(self, now_ns: int):
        status = self.gap_status(now_ns)
        if status == 'NO_MSG':
            self.no_msg_count += 1
            self.current_stale_streak = 0
        elif status == 'STALE':
            self.stale_count += 1
            self.current_stale_streak += 1
            self.max_stale_streak = max(self.max_stale_streak, self.current_stale_streak)
        else:
            self.current_stale_streak = 0
        return status


class ImuPipelineMonitor(Node):
    def __init__(self):
        super().__init__('imu_pipeline_monitor')

        topics = [
            ('/imu_broad/imu', 100.0, 'imu'),
            ('/imu/imu_corrected', 100.0, 'imu'),
            ('/imu/imu_corrected_deadband', 100.0, 'imu'),
            ('/imu/data', 100.0, 'imu'),
            ('/odometry/filtered', 100.0, 'odom'),
        ]
        self._stats = {name: TopicStats(name, hz, kind) for name, hz, kind in topics}

        for topic_name, _, kind in topics:
            if kind == 'imu':
                self.create_subscription(
                    Imu, topic_name, self._make_imu_cb(topic_name), 20)
            else:
                self.create_subscription(
                    Odometry, topic_name, self._make_odom_cb(topic_name), 20)

        self._timer = self.create_timer(0.5, self._print_dashboard)
        self._dashboard_lines = 15
        self._start_ns = self.get_clock().now().nanoseconds

        print('\n' * self._dashboard_lines, end='')
        self.get_logger().info('Monitoring IMU pipeline topics for dropouts and timing gaps')

    def _stamp_to_ns(self, stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _make_imu_cb(self, topic_name):
        def _cb(msg: Imu):
            now_ns = self.get_clock().now().nanoseconds
            stats = self._stats[topic_name]
            stats.update_common(now_ns, self._stamp_to_ns(msg.header.stamp))
            stats.last_wz = msg.angular_velocity.z
            stats.last_yaw_deg = quat_to_yaw_deg(msg.orientation)
        return _cb

    def _make_odom_cb(self, topic_name):
        def _cb(msg: Odometry):
            now_ns = self.get_clock().now().nanoseconds
            stats = self._stats[topic_name]
            stats.update_common(now_ns, self._stamp_to_ns(msg.header.stamp))
            stats.last_wz = msg.twist.twist.angular.z
            stats.last_yaw_deg = quat_to_yaw_deg(msg.pose.pose.orientation)
            stats.last_vx = msg.twist.twist.linear.x
        return _cb

    def _fmt_float(self, value, width=7, precision=3):
        if value is None:
            return f'{"-":>{width}}'
        return f'{value:>{width}.{precision}f}'

    def _print_dashboard(self):
        now_ns = self.get_clock().now().nanoseconds
        uptime_sec = max(0.0, (now_ns - self._start_ns) / 1e9)
        lines = [
            'IMU Pipeline Monitor',
            'topic                             st   hz   age_rx  age_ts  dt_last  dt_max    wz      yaw',
        ]
        for topic_name in [
            '/imu_broad/imu',
            '/imu/imu_corrected',
            '/imu/imu_corrected_deadband',
            '/imu/data',
            '/odometry/filtered',
        ]:
            stats = self._stats[topic_name]
            status = stats.update_status_counters(now_ns)
            line = (
                f'{topic_name:<32} '
                f'{status:<6}'
                f'{self._fmt_float(stats.avg_hz(), 6, 1)} '
                f'{self._fmt_float(stats.receive_age_ms(now_ns), 7, 1)} '
                f'{self._fmt_float(stats.stamp_age_ms(now_ns), 7, 1)} '
                f'{self._fmt_float(stats.last_interval_ms, 8, 1)} '
                f'{self._fmt_float(stats.max_interval_ms if stats.max_interval_ms > 0.0 else None, 7, 1)} '
                f'{self._fmt_float(stats.last_wz, 7, 4)} '
                f'{self._fmt_float(stats.last_yaw_deg, 8, 2)}'
            )
            if stats.kind == 'odom':
                line += f'  vx={self._fmt_float(stats.last_vx, 6, 3)}'
            lines.append(line)

        lines.append('')
        lines.append(f'运行时长: {uptime_sec:7.1f}s')
        lines.append('统计总结: topic                             >20ms  >30ms  >50ms  stale  no_msg  max_stale')
        for topic_name in [
            '/imu_broad/imu',
            '/imu/imu_corrected',
            '/imu/imu_corrected_deadband',
            '/imu/data',
            '/odometry/filtered',
        ]:
            stats = self._stats[topic_name]
            lines.append(
                f'          {topic_name:<32} '
                f'{stats.over_20ms_count:>5} '
                f'{stats.over_30ms_count:>6} '
                f'{stats.over_50ms_count:>6} '
                f'{stats.stale_count:>6} '
                f'{stats.no_msg_count:>7} '
                f'{stats.max_stale_streak:>10}'
            )

        lines.append('')
        lines.append('判断方法: st=NO_MSG 表示没收到; st=STALE 表示超过 2.5 个周期未更新。')
        lines.append('重点对比: corrected 是否低于 broad, deadband 的 wz 是否被压零, /imu/data 与 /odometry/filtered 是否持续更新。')
        lines.append('(Ctrl+C 退出)')

        self._dashboard_lines = len(lines)
        sys.stdout.write(f'\033[{self._dashboard_lines}A\033[J')
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ImuPipelineMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sys.stdout.write('\nMonitor stopped.\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
