#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import sys

class DataMonitor(Node):
    def __init__(self):
        super().__init__('data_monitor')
        
        # 订阅核心话题
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)

        # 原始 Odom 数据
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0
        
        # 原始 IMU 数据
        self.imu_wz = 0.0
        self.imu_yaw = 0.0
        
        # EKF 数据
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_yaw = 0.0
        self.ekf_vx = 0.0
        self.ekf_wz = 0.0

        # 创建定时器，10Hz 刷新一次界面
        self.timer = self.create_timer(0.1, self.print_dashboard)
        
        # 预留打印空间 (增加到 8 行以容纳更多信息)
        print('\n' * 8)

    def get_yaw_from_quat(self, q):
        """将四元数转换为偏航角 Yaw (角度制)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # return math.degrees(math.atan2(siny_cosp, cosy_cosp))

        yaw_degrees = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return yaw_degrees % 360.0

    def odom_cb(self, msg):
        # 提取坐标 (m -> cm)
        self.odom_x = msg.pose.pose.position.x * 100.0
        self.odom_y = msg.pose.pose.position.y * 100.0
        # 提取线速度和角速度
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        # 提取偏航角
        self.odom_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def imu_cb(self, msg):
        self.imu_wz = msg.angular_velocity.z
        self.imu_yaw = self.get_yaw_from_quat(msg.orientation)

    def ekf_cb(self, msg):
        # 提取位置并转换为厘米 (m -> cm)
        self.ekf_x = msg.pose.pose.position.x * 100.0
        self.ekf_y = msg.pose.pose.position.y * 100.0
        # 提取速度
        self.ekf_vx = msg.twist.twist.linear.x
        self.ekf_wz = msg.twist.twist.angular.z
        # 提取偏航角
        self.ekf_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def print_dashboard(self):
        # ANSI 转义码：\033[8A 光标向上移动8行
        sys.stdout.write('\033[8A\033[J')
        
        dashboard = (
            f"   === 机器人底盘 & EKF 实时数据监控 ===\n"
            f"   [原始 Odom] 坐标: X:{self.odom_x:>6.1f}cm, Y:{self.odom_y:>6.1f}cm, Yaw:{self.odom_yaw:>7.2f}°\n"
            f"   [原始 Odom] 速度: vx:{self.odom_vx:>6.3f}m/s, wz:{self.odom_wz:>6.3f}rad/s\n"
            f"   [原始 IMU ] 偏航: {self.imu_yaw:>7.2f} °, 角速度(wz): {self.imu_wz:>7.3f} rad/s\n"
            f"   -------------------------------------------------\n"
            f"   [融合 EKF ] 坐标: X:{self.ekf_x:>6.1f}cm, Y:{self.ekf_y:>6.1f}cm, Yaw:{self.ekf_yaw:>7.2f}°\n"
            f"   [融合 EKF ] 速度: vx:{self.ekf_vx:>6.3f}m/s, wz:{self.ekf_wz:>6.3f}rad/s\n"
            f"   =================================================\n"
            f"   (按 Ctrl+C 退出)"
        )
        sys.stdout.write(dashboard)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DataMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sys.stdout.write('\n\n监控已停止。\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()