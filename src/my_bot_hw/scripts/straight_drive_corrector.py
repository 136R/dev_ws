#!/usr/bin/env python3
# Copyright 2024 my_bot_hw
# SPDX-License-Identifier: Apache-2.0
"""
straight_drive_corrector.py

Inserts a gyro-based yaw correction when driving straight.
Pass-through when turning (|angular.z| >= straight_threshold).

Topic flow:
  /cmd_vel_mux_out  ──┐
  /imu/data_raw     ──┤── [this node] ──→ /diff_cont/cmd_vel_unstamped

Parameters:
  straight_threshold  (float, rad/s)  : angular.z below this = straight intent  [0.05]
  kp                  (float)         : proportional gain on gyro_z error        [0.5]
  max_correction      (float, rad/s)  : clamp correction to ±this value          [0.3]
  min_linear          (float, m/s)    : only correct when moving faster than this [0.01]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class StraightDriveCorrector(Node):
    def __init__(self):
        super().__init__('straight_drive_corrector')

        self.declare_parameter('straight_threshold', 0.05)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('max_correction', 0.8)
        self.declare_parameter('min_linear', 0.05)
        self.declare_parameter('imu_topic', '/imu/data_raw')

        self._threshold   = self.get_parameter('straight_threshold').value
        self._kp          = self.get_parameter('kp').value
        self._max_corr    = self.get_parameter('max_correction').value
        self._min_linear  = self.get_parameter('min_linear').value
        imu_topic         = self.get_parameter('imu_topic').value

        self._gyro_z = 0.0

        self._sub_imu = self.create_subscription(Imu,   imu_topic,    self._imu_cb, 10)
        self._sub_cmd = self.create_subscription(Twist, 'cmd_vel_in', self._cmd_cb, 10)
        self._pub     = self.create_publisher(   Twist, 'cmd_vel_out', 10)

        self.get_logger().info(
            f'straight_drive_corrector: kp={self._kp}  '
            f'threshold={self._threshold} rad/s  '
            f'max_correction=±{self._max_corr} rad/s  '
            f'imu_topic={imu_topic}')

    def _imu_cb(self, msg: Imu):
        self._gyro_z = msg.angular_velocity.z

    def _cmd_cb(self, msg: Twist):
        out = Twist()
        out.linear = msg.linear

        straight_intent = abs(msg.angular.z) < self._threshold
        moving          = abs(msg.linear.x)  > self._min_linear

        if straight_intent and moving:
            # Desired yaw rate = 0; actual = gyro_z → error = -gyro_z
            correction = max(-self._max_corr,
                             min(self._max_corr, -self._kp * self._gyro_z))
            out.angular.z = msg.angular.z + correction
        else:
            out.angular.z = msg.angular.z

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StraightDriveCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
