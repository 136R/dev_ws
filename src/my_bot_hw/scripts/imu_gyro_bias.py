#!/usr/bin/env python3
# imu_gyro_bias.py — Gyro bias calibration node
#
# Subscribes to raw IMU, collects gyro_calib_samples frames while the robot is
# stationary, computes the mean as bias, then publishes bias-corrected IMU.
# Passthrough mode: accel and orientation are forwarded unchanged.
#
# Topics (remappable via parameters):
#   input_topic  (default /imu_broad/imu)  → raw IMU
#   output_topic (default /imu/imu_corrected) → bias-corrected IMU

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu


class ImuGyroBias(Node):
    def __init__(self):
        super().__init__('imu_gyro_bias')

        self.declare_parameter('gyro_calib_samples', 300)
        self.declare_parameter('input_topic',  '/imu_broad/imu')
        self.declare_parameter('output_topic', '/imu/imu_corrected')

        self._n_samples   = int(self.get_parameter('gyro_calib_samples').value)
        input_topic       = self.get_parameter('input_topic').value
        output_topic      = self.get_parameter('output_topic').value

        self._count = 0
        self._sum   = [0.0, 0.0, 0.0]
        self._bias  = [0.0, 0.0, 0.0]
        self._calibrated = False

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Imu, output_topic, 10)
        self._sub = self.create_subscription(
            Imu, input_topic, self._cb, sensor_qos)

        self.get_logger().info(
            f'陀螺仪零偏校准: 输入={input_topic} 输出={output_topic} '
            f'采样帧数={self._n_samples} — 请保持机器人静止')

    # ------------------------------------------------------------------
    def _cb(self, msg: Imu):
        if not self._calibrated:
            self._sum[0] += msg.angular_velocity.x
            self._sum[1] += msg.angular_velocity.y
            self._sum[2] += msg.angular_velocity.z
            self._count  += 1

            if self._count == 1:
                self.get_logger().info('开始采集陀螺仪零偏样本，请保持静止...')

            if self._count >= self._n_samples:
                self._bias = [s / self._count for s in self._sum]
                self._calibrated = True
                self.get_logger().info(
                    f'零偏校准完成  bias=[{self._bias[0]:.6f}, '
                    f'{self._bias[1]:.6f}, {self._bias[2]:.6f}] rad/s')
            return   # 校准阶段不发布

        # out = copy.deepcopy(msg)
        # out.angular_velocity.x -= self._bias[0]
        # out.angular_velocity.y -= self._bias[1]
        # out.angular_velocity.z -= self._bias[2]
        # self._pub.publish(out)

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # 只针对需要修改的部分进行计算
        out.angular_velocity.x = msg.angular_velocity.x - self._bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - self._bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - self._bias[2]

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGyroBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
