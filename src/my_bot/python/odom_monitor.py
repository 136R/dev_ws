#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10
        )
        print("✅ 已订阅 /odometry/filtered，等待数据...\n")

    def callback(self, msg):
        x = msg.pose.pose.position.x * 100.0
        y = msg.pose.pose.position.y * 100.0

        q = msg.pose.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        yaw_deg = math.degrees(yaw_rad)

        print(f"\rX: {x:8.2f} cm  |  Y: {y:8.2f} cm  |  Z(偏航): {yaw_deg:8.2f}°", end='', flush=True)

def main():
    rclpy.init()
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 已停止监控")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()