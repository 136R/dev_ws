#!/usr/bin/env python3
# Copyright 2024 my_bot_hw
# SPDX-License-Identifier: Apache-2.0

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuGyroDeadband(Node):
    def __init__(self):
        super().__init__('imu_gyro_deadband')

        self.declare_parameter('deadband_z', 0.0002)
        self.declare_parameter('gyro_z_scale', 1.0)
        self.declare_parameter('input_topic', '/imu/imu_corrected')
        self.declare_parameter('output_topic', '/imu/imu_corrected_deadband')

        self._deadband_z = float(self.get_parameter('deadband_z').value)
        self._gyro_z_scale = float(self.get_parameter('gyro_z_scale').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(Imu, output_topic, 10)
        self._sub = self.create_subscription(Imu, input_topic, self._imu_cb, 10)

        self.get_logger().info(
            f'IMU陀螺仪处理: 输入={input_topic} 输出={output_topic} '
            f'Z轴死区={self._deadband_z:.6f} rad/s '
            f'gyro_z_scale={self._gyro_z_scale:.6f}'
        )

    def _imu_cb(self, msg: Imu):
        out = copy.deepcopy(msg)
        out.angular_velocity.z *= self._gyro_z_scale
        if abs(out.angular_velocity.z) < self._deadband_z:
            out.angular_velocity.z = 0.0
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGyroDeadband()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
