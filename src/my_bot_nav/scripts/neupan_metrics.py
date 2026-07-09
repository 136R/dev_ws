#!/usr/bin/env python3
"""NeuPAN 测试指标记录器（Phase 5）。

订阅 /scan 与 TF(map->base_footprint)，记录一次导航从当前位置到给定目标的：
  - 到达与否（进入 arrive_radius 即算到达）
  - 用时、路径长度、平均速度
  - 全程最小障碍距离（激光最近返回，近似 clearance）
  - /cmd_vel_nav 控制频率

用法：
  ros2 run my_bot_nav neupan_metrics.py --ros-args -p goal_x:=5.5 -p goal_y:=0.0 \
        -p arrive_radius:=0.2 -p timeout:=60.0
运行前请先发目标（RViz 2D Goal Pose 或 topic pub /goal_pose）。
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros


class Metrics(Node):
    def __init__(self):
        super().__init__("neupan_metrics")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gx = self.declare_parameter("goal_x", 5.5).value
        self.gy = self.declare_parameter("goal_y", 0.0).value
        self.arrive_r = self.declare_parameter("arrive_radius", 0.2).value
        self.timeout = self.declare_parameter("timeout", 60.0).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_cb, 10)

        self.min_clear = math.inf
        self.cmd_count = 0
        self.path_len = 0.0
        self.prev_xy = None
        self.start_xy = None
        self.t0 = None
        self.arrived = False
        # 摆动统计（基于 /cmd_vel_nav 的 angular.z）
        self.wz_sum = 0.0
        self.wz_sqsum = 0.0
        self.wz_n = 0
        self.wz_prev_sign = 0
        self.wz_flips = 0
        self.wz_deadband = 0.02  # rad/s，低于此视作 0，避免噪声误计变号
        self.timer = self.create_timer(0.1, self.tick)

    def scan_cb(self, msg):
        rs = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
        if rs:
            self.min_clear = min(self.min_clear, min(rs))

    def cmd_cb(self, msg):
        self.cmd_count += 1
        wz = msg.angular.z
        self.wz_sum += wz
        self.wz_sqsum += wz * wz
        self.wz_n += 1
        sign = 0 if abs(wz) < self.wz_deadband else (1 if wz > 0 else -1)
        if sign != 0:
            if self.wz_prev_sign != 0 and sign != self.wz_prev_sign:
                self.wz_flips += 1
            self.wz_prev_sign = sign

    def pose(self):
        try:
            t = self.tfbuf.lookup_transform(self.map_frame, self.base_frame,
                                            rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def tick(self):
        p = self.pose()
        if p is None:
            return
        if self.t0 is None:
            self.t0 = time.time()
            self.prev_xy = p
            self.start_xy = p
            return
        dx, dy = p[0] - self.prev_xy[0], p[1] - self.prev_xy[1]
        self.path_len += math.hypot(dx, dy)
        self.prev_xy = p
        dist_goal = math.hypot(p[0] - self.gx, p[1] - self.gy)
        el = time.time() - self.t0
        if dist_goal <= self.arrive_r and not self.arrived:
            self.arrived = True
            self.finish(el, "ARRIVED")
        elif el > self.timeout:
            self.finish(el, "TIMEOUT")

    def finish(self, el, status):
        avg_v = self.path_len / el if el > 0 else 0.0
        rate = self.cmd_count / el if el > 0 else 0.0
        # 摆动评分
        if self.wz_n > 0:
            wz_mean = self.wz_sum / self.wz_n
            wz_var = max(0.0, self.wz_sqsum / self.wz_n - wz_mean * wz_mean)
            wz_std = math.sqrt(wz_var)
        else:
            wz_std = 0.0
        flips_per_m = self.wz_flips / self.path_len if self.path_len > 0.05 else 0.0
        # 路径效率比 = 实走路长 / 起终点直线距离（越接近 1 越直，越大越绕/画龙）
        if self.start_xy is not None:
            straight = math.hypot(self.gx - self.start_xy[0], self.gy - self.start_xy[1])
            eff_ratio = self.path_len / straight if straight > 0.05 else float("nan")
        else:
            straight, eff_ratio = float("nan"), float("nan")
        print("\n================ NeuPAN 测试指标 ================")
        print(f" 结果        : {status}")
        print(f" 目标        : ({self.gx:.2f}, {self.gy:.2f}) arrive_r={self.arrive_r}")
        print(f" 用时        : {el:.1f} s")
        print(f" 路径长度    : {self.path_len:.2f} m")
        print(f" 平均速度    : {avg_v:.3f} m/s")
        print(f" 最小障碍距离: {self.min_clear:.3f} m  (激光最近返回)")
        print(f" cmd_vel频率 : {rate:.1f} Hz")
        print(" ---- 画龙/摆动评分 ----")
        print(f" 角速度变号  : {self.wz_flips} 次, {flips_per_m:.2f} 次/m  (越小越不画龙)")
        print(f" 角速度标准差: {wz_std:.3f} rad/s  (越小越平顺)")
        print(f" 路径效率比  : {eff_ratio:.2f}  (直线={straight:.2f}m, 1.0=完全直, 越大越绕)")
        print("===============================================\n")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Metrics()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
