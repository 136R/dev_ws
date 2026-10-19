#!/usr/bin/env python3
"""按"车中心格子"判断路线还能不能走，替代 planner_server 的 /is_path_valid。

planner_server 自带的检查用整个车身轮廓 + footprint_padding 去碰墙格子，SmacPlanner2D 只看
中心格子 —— 标准不一致，刚规划出的路线（尤其过 40cm 口子）就被判撞墙，BT 每秒都重规划。
这里改成和规划器同一标准：从车最近的路线点往后，中心格子是墙或贴墙区（>=253）就判走不通。

bt_navigator 启动时传 path_valid_service:=/path_center_check，BT 里的 IsPathValid 就会来问这里。
"""

import rclpy
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

INSCRIBED = 253  # 253 贴墙区、254 墙；255 未知不算（与 allow_unknown: true 一致）
NO_INFO = 255


class PathCenterCheck(Node):
    def __init__(self):
        super().__init__('path_center_check')
        self.costmap = None
        self.buf = Buffer()
        TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self.costmap_cb, qos)
        self.create_service(IsPathValid, '/path_center_check', self.check)

    def costmap_cb(self, msg):
        self.costmap = msg

    def check(self, req, res):
        res.is_valid = False
        poses = req.path.poses
        m = self.costmap
        if not poses or m is None:
            return res

        # 从离车最近的路线点开始查，走过的不管；查不到车位置就从头查
        start = 0
        try:
            tf = self.buf.lookup_transform(m.header.frame_id, 'base_footprint', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            start = min(range(len(poses)), key=lambda i: (poses[i].pose.position.x - rx) ** 2
                        + (poses[i].pose.position.y - ry) ** 2)
        except Exception:
            pass

        res_m, ox, oy = m.metadata.resolution, m.metadata.origin.position.x, m.metadata.origin.position.y
        w, h = m.metadata.size_x, m.metadata.size_y
        for p in poses[start:]:
            i = int((p.pose.position.x - ox) / res_m)
            j = int((p.pose.position.y - oy) / res_m)
            if not (0 <= i < w and 0 <= j < h):
                return res
            c = m.data[j * w + i]
            if INSCRIBED <= c < NO_INFO:
                return res
        res.is_valid = True
        return res


def main():
    rclpy.init()
    node = PathCenterCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
