#!/usr/bin/env python3
"""
Gazebo 地面真值 vs 里程计 对比工具
用法：python3 gz_pose_monitor.py

依赖：gz_bridge.yaml 中需要添加以下桥接
  - ros_topic_name: /gz/model/my_bot/pose
    gz_topic_name: /model/my_bot/pose
    ros_type_name: geometry_msgs/msg/PoseStamped
    gz_type_name: gz.msgs.Pose
    direction: GZ_TO_ROS
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
import math


def quat_to_yaw_deg(x, y, z, w):
    """四元数转偏航角（度）"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class PoseMonitor(Node):

    # ── 修改这里换机器人名称 ──────────────────
    ROBOT_MODEL_NAME = "my_bot"
    # ──────────────────────────────────────────

    def __init__(self):
        super().__init__('gz_pose_monitor')

        self.gz_x   = None
        self.gz_y   = None
        self.gz_yaw = None
        self.gz_source = "未连接"

        self.odom_x   = None
        self.odom_y   = None
        self.odom_yaw = None

        # 优先：单独话题（精确，按名称）
        self.create_subscription(
            PoseStamped,
            '/gz/model/my_bot/pose',
            self.gz_single_pose_callback,
            10
        )

        # 备用：PoseArray index=9（my_bot 是世界中第10个模型）
        self.create_subscription(
            PoseArray,
            '/world_poses',
            self.gz_posearray_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_timer(0.5, self.print_comparison)
        self.get_logger().info(
            f'Pose Monitor 启动，监控机器人：{self.ROBOT_MODEL_NAME}'
        )

    def gz_single_pose_callback(self, msg: PoseStamped):
        """优先使用单独话题（按名称桥接，最精确）"""
        p = msg.pose.position
        o = msg.pose.orientation
        self.gz_x      = p.x
        self.gz_y      = p.y
        self.gz_yaw    = quat_to_yaw_deg(o.x, o.y, o.z, o.w)
        self.gz_source = "/gz/model/my_bot/pose"

    def gz_posearray_callback(self, msg: PoseArray):
        """备用：从 PoseArray 取 index=9"""
        if self.gz_source == "/gz/model/my_bot/pose":
            return   # 已有精确来源，跳过
        idx = 9      # my_bot 在世界中的顺序（0-based）
        if len(msg.poses) <= idx:
            self.get_logger().warn(
                f'PoseArray 只有 {len(msg.poses)} 个元素，'
                f'期望 index={idx}', throttle_duration_sec=5.0
            )
            return
        pose = msg.poses[idx]
        self.gz_x      = pose.position.x
        self.gz_y      = pose.position.y
        self.gz_yaw    = quat_to_yaw_deg(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        )
        self.gz_source = f"/world_poses[{idx}]"

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.odom_x   = p.x
        self.odom_y   = p.y
        self.odom_yaw = quat_to_yaw_deg(o.x, o.y, o.z, o.w)

    @staticmethod
    def _bar(val, max_val=0.20, width=20):
        filled = int(min(val / max_val, 1.0) * width)
        return '█' * filled + '░' * (width - filled)

    def print_comparison(self):
        print('\033c', end='')
        print('=' * 58)
        print('        Gazebo 地面真值 vs 里程计 对比工具')
        print('=' * 58)

        # Gazebo 真值
        if self.gz_x is not None:
            print(f'\n【Gazebo 地面真值】  来源：{self.gz_source}')
            print(f'  X：{self.gz_x:+10.4f} m')
            print(f'  Y：{self.gz_y:+10.4f} m')
            print(f'  Z（偏航）：{self.gz_yaw:+8.2f} °')
        else:
            print('\n【Gazebo 地面真值】等待数据...')
            print('  请确认 gz_bridge.yaml 已添加 /gz/model/my_bot/pose')
            print('  或 /world_poses (PoseArray) 桥接')

        # 里程计
        if self.odom_x is not None:
            print(f'\n【里程计 /odom】')
            print(f'  X：{self.odom_x:+10.4f} m')
            print(f'  Y：{self.odom_y:+10.4f} m')
            print(f'  Z（偏航）：{self.odom_yaw:+8.2f} °')
        else:
            print('\n【里程计 /odom】等待数据...')

        # 误差分析
        if self.gz_x is not None and self.odom_x is not None:
            dx       = self.gz_x - self.odom_x
            dy       = self.gz_y - self.odom_y
            dist_err = math.sqrt(dx**2 + dy**2)
            dyaw     = abs(self.gz_yaw - self.odom_yaw)
            if dyaw > 180:
                dyaw = 360 - dyaw

            print(f'\n【误差分析】')
            print(f'  ΔX：{dx:+9.4f} m')
            print(f'  ΔY：{dy:+9.4f} m')
            print(f'  位置误差：{dist_err:7.4f} m  '
                  f'{self._bar(dist_err)}  ', end='')
            if dist_err < 0.05:
                print('✅ 优秀（< 5cm）')
            elif dist_err < 0.10:
                print('🟡 一般（< 10cm）')
            else:
                print('🔴 偏大（> 10cm）')

            print(f'  偏航误差：{dyaw:7.2f} °    '
                  f'{self._bar(dyaw, max_val=10.0)}  ', end='')
            if dyaw < 2.0:
                print('✅ 优秀（< 2°）')
            elif dyaw < 5.0:
                print('🟡 一般（< 5°）')
            else:
                print('🔴 偏大（> 5°）')

        print('\n' + '=' * 58)
        print('Ctrl+C 退出')


def main():
    rclpy.init()
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()