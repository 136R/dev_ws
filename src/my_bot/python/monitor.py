#!/usr/bin/env python3
"""仿真数据监控：轮式里程计、融合 EKF 偏航、Gazebo 地面真值。"""

import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class DataMonitor(Node):
    ROBOT_MODEL_NAME = 'my_bot'
    WORLD_POSES_INDEX = 9
    GZ_SINGLE_POSE_TOPIC = '/gz/model/my_bot/pose'
    GZ_WORLD_POSES_TOPIC = '/world_poses'
    ODOM_TOPIC = '/odom'
    EKF_TOPIC = '/odometry/filtered'
    CMD_VEL_NAV2_TOPIC = '/cmd_vel_nav2'

    def __init__(self):
        super().__init__('data_monitor')

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.odom_vx = None
        self.odom_wz = None

        self.ekf_x = None
        self.ekf_y = None
        self.ekf_yaw = None
        self.ekf_vx = None
        self.ekf_wz = None

        self.cmd_nav2_vx = None
        self.cmd_nav2_wz = None

        self.gz_x = None
        self.gz_y = None
        self.gz_yaw = None
        self.gz_source = '未连接'

        self.map_x = None
        self.map_y = None
        self.map_yaw = None
        self.map_status = '等待 TF(map->base_footprint)...'

        self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(Odometry, self.EKF_TOPIC, self.ekf_cb, 10)
        self.create_subscription(Twist, self.CMD_VEL_NAV2_TOPIC, self.cmd_nav2_cb, 10)
        self.create_subscription(
            PoseStamped,
            self.GZ_SINGLE_POSE_TOPIC,
            self.gz_single_pose_cb,
            10,
        )
        self.create_subscription(
            PoseArray,
            self.GZ_WORLD_POSES_TOPIC,
            self.gz_posearray_cb,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.2, self.print_dashboard)
        self.get_logger().info(
            f'仿真监控启动：{self.ODOM_TOPIC}, {self.EKF_TOPIC}, '
            f'{self.CMD_VEL_NAV2_TOPIC}, '
            f'{self.GZ_SINGLE_POSE_TOPIC}/{self.GZ_WORLD_POSES_TOPIC}'
        )
        print('\n' * 14)

    @staticmethod
    def normalize_angle_degrees(angle):
        return (angle + 180.0) % 360.0 - 180.0

    @classmethod
    def quat_to_yaw_deg(cls, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return cls.normalize_angle_degrees(math.degrees(math.atan2(siny_cosp, cosy_cosp)))

    @staticmethod
    def angle_error_deg(reference, measured):
        return abs((reference - measured + 180.0) % 360.0 - 180.0)

    def odom_cb(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        self.odom_yaw = self.quat_to_yaw_deg(pose.orientation)
        self.odom_vx = twist.linear.x
        self.odom_wz = twist.angular.z

    def ekf_cb(self, msg):
        pose = msg.pose.pose
        self.ekf_x = pose.position.x
        self.ekf_y = pose.position.y
        self.ekf_yaw = self.quat_to_yaw_deg(pose.orientation)
        self.ekf_vx = msg.twist.twist.linear.x
        self.ekf_wz = msg.twist.twist.angular.z

    def cmd_nav2_cb(self, msg):
        self.cmd_nav2_vx = msg.linear.x
        self.cmd_nav2_wz = msg.angular.z

    def gz_single_pose_cb(self, msg):
        pose = msg.pose
        self.gz_x = pose.position.x
        self.gz_y = pose.position.y
        self.gz_yaw = self.quat_to_yaw_deg(pose.orientation)
        self.gz_source = self.GZ_SINGLE_POSE_TOPIC

    def gz_posearray_cb(self, msg):
        if self.gz_source == self.GZ_SINGLE_POSE_TOPIC:
            return

        if len(msg.poses) <= self.WORLD_POSES_INDEX:
            self.get_logger().warn(
                f'PoseArray 只有 {len(msg.poses)} 个元素，'
                f'期望 index={self.WORLD_POSES_INDEX}',
                throttle_duration_sec=5.0,
            )
            return

        pose = msg.poses[self.WORLD_POSES_INDEX]
        self.gz_x = pose.position.x
        self.gz_y = pose.position.y
        self.gz_yaw = self.quat_to_yaw_deg(pose.orientation)
        self.gz_source = f'{self.GZ_WORLD_POSES_TOPIC}[{self.WORLD_POSES_INDEX}]'

    def update_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.map_x = None
            self.map_y = None
            self.map_yaw = None
            self.map_status = '等待 TF(map->base_footprint)...'
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.map_x = translation.x
        self.map_y = translation.y
        self.map_yaw = self.quat_to_yaw_deg(rotation)
        self.map_status = 'TF: map -> base_footprint'

    @staticmethod
    def pose_line(x, y, yaw):
        if x is None:
            return '等待数据...'
        return f'X:{x:+8.4f} m, Y:{y:+8.4f} m, Yaw:{yaw:+8.2f}°'

    @staticmethod
    def velocity_line(vx, wz):
        if vx is None:
            return '等待数据...'
        return f'vx:{vx:+7.3f} m/s, wz:{wz:+7.3f} rad/s'

    def print_dashboard(self):
        self.update_map_pose()
        sys.stdout.write('\033[14A\033[J')
        dashboard = (
            '   === 仿真话题实时监控：轮式里程计 / 融合 EKF 偏航 / Gazebo 地面真值 ===\n'
            f'   [轮式里程计 {self.ODOM_TOPIC:<20}] {self.pose_line(self.odom_x, self.odom_y, self.odom_yaw)}\n'
            f'   [轮式里程计 速度                 ] {self.velocity_line(self.odom_vx, self.odom_wz)}\n'
            f'   [融合 EKF {self.EKF_TOPIC:<25}] {self.pose_line(self.ekf_x, self.ekf_y, self.ekf_yaw)}\n'
            f'   [融合 EKF 速度                  ] {self.velocity_line(self.ekf_vx, self.ekf_wz)}\n'
            f'   [MPPI 输出 {self.CMD_VEL_NAV2_TOPIC:<23}] {self.velocity_line(self.cmd_nav2_vx, self.cmd_nav2_wz)}\n'
            f'   [Map 位姿 map->base_footprint   ] {self.pose_line(self.map_x, self.map_y, self.map_yaw)}\n'
            f'   [Map 状态                       ] {self.map_status}\n'
            '   ---------------------------------------------------------------\n'
            f'   [Gazebo 地面真值                 ] {self.pose_line(self.gz_x, self.gz_y, self.gz_yaw)}\n'
            f'   [Gazebo 来源                     ] {self.gz_source}\n'
            '   ---------------------------------------------------------------\n'
            f'   备用真值：{self.GZ_WORLD_POSES_TOPIC}[{self.WORLD_POSES_INDEX}]；优先真值：{self.GZ_SINGLE_POSE_TOPIC}\n'
            '   Ctrl+C 退出'
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
