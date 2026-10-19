#!/usr/bin/env python3
"""仿真导航跑圈测试（my_world_40）。

按固定路线连续下发 NavigateToPose（穿过各 40cm 口子、两个死胡同，最后回原点），
每 0.05s 记录一次：
  - 跟踪偏差：车中心到当前 /plan 的距离
  - 车身离墙：按真实车身外廓到 sdf 真实墙面的距离（负数 = 已擦墙）
  - planner 报 "Starting point in lethal space" 的次数
某段失败时自动脱困（/cmd_vel_keyboard 挪开 → padding 临时降到 0.01 开回原点 → 恢复 0.05），再开下一圈。

用法（仿真、定位、导航都已起来，车停在原点）：
  ros2 run my_bot_nav nav_route_test.py B '{"min_lookahead_dist":0.15,"max_lookahead_dist":0.3}' 3
  ros2 run my_bot_nav nav_route_test.py cur '{}' 1     # {} = 不改参数，用控制器当前值
  ros2 run my_bot_nav nav_route_test.py r2 '{}' 1 --route '[[1.6,-0.6],[0.0,0.0]]'
参数：<标签> <FollowPath 参数 JSON> <圈数> [--world sdf] [--route JSON]
--route 要从原点出发、最后回原点（失败脱困也是开回原点）。只能在仿真跑（要 sdf 真值、会自动发键盘速度）。
结果写到当前目录 <标签>_all.json（每段轨迹）、<标签>_fails.json（失败段）。
"""
import argparse
import json
import math
import os
import re
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

WORLD = os.path.join(get_package_share_directory("my_bot"), "worlds", "my_world_40.sdf")
ROUTE = [(0.99, -1.71), (0.0, 0.0), (1.6, -0.6), (0.0, 1.55), (-1.65, 0.75), (-1.5, -1.25), (0.0, 0.0)]
# 裸车身外廓（base_link 系），与 my_bot_hw/config/laser_filters.yaml 的 body_polygon_filter 同一份
BODY = [[0.063, 0.0615], [0.035, 0.11], [-0.0765, 0.11], [-0.1275, 0.0805], [-0.157, 0.0295],
        [-0.157, -0.0295], [-0.1275, -0.0805], [-0.0765, -0.11], [0.035, -0.11], [0.063, -0.0615]]
PAD_NORMAL = 0.05   # 全局 costmap 平时的 footprint_padding
PAD_RECOVER = 0.01  # 脱困时临时值，让 planner 能从贴墙位置出发


def load_walls(path):
    """sdf 墙体 → 轴对齐矩形列表 (xmin, xmax, ymin, ymax)。只处理外墙 + 0/90° 的 obstacle_* 墙。"""
    s = open(path).read()
    walls = [(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)
             for x, y, sx, sy in [(2, 0, 0.1, 4), (0, 2, 4, 0.1), (-2, 0, 0.1, 4), (0, -2, 4, 0.1)]]
    for m in re.finditer(r'<link name="(obstacle_[^"]+)">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', s, re.S):
        x, y, _, _, _, yaw = map(float, m.group(2).split())
        sx, sy, _ = map(float, m.group(3).split())
        if abs(abs(yaw) - 1.5708) < 0.01:
            sx, sy = sy, sx
        walls.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return walls


WALLS = []  # main() 里按 --world 加载


def sdist(x, y):
    """点到最近墙面的有符号距离（在墙内为负）。"""
    best = 9.0
    for a, b, c, d in WALLS:
        dx = max(a - x, 0, x - b)
        dy = max(c - y, 0, y - d)
        if dx == 0 and dy == 0:
            best = min(best, -min(x - a, b - x, y - c, d - y))
        else:
            best = min(best, math.hypot(dx, dy))
    return best


# 车身外廓每 1cm 取一个采样点
SAMP = []
for i in range(len(BODY)):
    a, b = BODY[i], BODY[(i + 1) % len(BODY)]
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.01))
    SAMP += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]


def body_clear(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return min(sdist(x + c * px - s * py, y + s * px + c * py) for px, py in SAMP)


def xtrack(x, y, path):
    """点到折线的最近距离。"""
    best = 9.0
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0 if l2 == 0 else max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / l2))
        best = min(best, math.hypot(ax + t * dx - x, ay + t * dy - y))
    return best


class RouteTester(Node):
    def __init__(self):
        super().__init__("nav_route_test")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan = None
        self.lethal = 0
        self.trace = []
        self.create_subscription(Path, "/plan", self.plan_cb, 10)
        self.create_subscription(Log, "/rosout", self.log_cb, 50)
        self.ctrl_cli = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.cm_cli = self.create_client(SetParameters, "/global_costmap/global_costmap/set_parameters")
        self.kb_pub = self.create_publisher(Twist, "/cmd_vel_keyboard", 10)

    def plan_cb(self, m):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def log_cb(self, m):
        if "lethal" in m.msg:
            self.lethal += 1

    def set_params(self, cli, params):
        if not params:
            return []
        req = SetParameters.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        return [r.successful for r in f.result().results]

    def pose(self, tries=400):
        for _ in range(tries):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                q = tf.transform.rotation
                return tf.transform.translation.x, tf.transform.translation.y, 2 * math.atan2(q.z, q.w)
            except Exception:
                pass
        return None

    def go(self, gx, gy, tmax=60):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = gx
        g.pose.pose.position.y = gy
        g.pose.pose.orientation.w = 1.0
        self.plan, self.lethal, self.trace = None, 0, []
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        rf = h.get_result_async()
        t0 = time.time()
        xt, bc = [], []
        while not rf.done() and time.time() - t0 < tmax:
            p = self.pose(tries=1)
            if p is None:
                continue
            x, y, th = p
            b = body_clear(x, y, th)
            bc.append(b)
            e = xtrack(x, y, self.plan) if self.plan and len(self.plan) > 1 else -1
            if e >= 0:
                xt.append(e)
            self.trace.append((round(time.time() - t0, 2), round(x, 3), round(y, 3),
                               round(math.degrees(th)), round(e * 100, 1), round(b * 100, 1)))
        if not rf.done():
            cf = h.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cf, timeout_sec=5)
            st = "TIMEOUT"
        else:
            st = {GoalStatus.STATUS_SUCCEEDED: "OK", GoalStatus.STATUS_ABORTED: "ABORT",
                  GoalStatus.STATUS_CANCELED: "CANCEL"}.get(rf.result().status, str(rf.result().status))
        end = self.pose(tries=40) or (float("nan"), float("nan"), 0)
        xt.sort()
        return dict(goal=(gx, gy), status=st, t=round(time.time() - t0, 1),
                    xt_max=round(xt[-1] * 100, 1) if xt else 0,
                    xt_p95=round(xt[int(len(xt) * 0.95)] * 100, 1) if xt else 0,
                    body_min=round(min(bc or [9]) * 100, 1), lethal=self.lethal,
                    end=(round(end[0], 2), round(end[1], 2)))

    def unstick(self):
        """按 1.5s 后车身离墙最大的方向，用键盘话题把车挪开。"""
        x, y, th = self.pose()
        best = None
        for v, w in [(-0.1, 0), (0.1, 0), (0, 0.8), (0, -0.8), (-0.1, 0.8), (-0.1, -0.8), (0.1, 0.8), (0.1, -0.8)]:
            xx, yy, tt = x, y, th
            for _ in range(30):
                tt += w * 0.05
                xx += v * math.cos(tt) * 0.05
                yy += v * math.sin(tt) * 0.05
            c = body_clear(xx, yy, tt)
            if best is None or c > best[0]:
                best = (c, v, w)
        t = Twist()
        t.linear.x, t.angular.z = float(best[1]), float(best[2])
        t0 = time.time()
        while time.time() - t0 < 1.5:
            self.kb_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.kb_pub.publish(Twist())
        time.sleep(0.5)

    def recover(self):
        self.unstick()
        self.set_params(self.cm_cli, {"footprint_padding": PAD_RECOVER})
        time.sleep(3)
        r = self.go(0.0, 0.0)
        self.set_params(self.cm_cli, {"footprint_padding": PAD_NORMAL})
        time.sleep(3)
        return r["status"] == "OK"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("params", help="FollowPath 参数 JSON，{} = 不改")
    ap.add_argument("reps", type=int)
    ap.add_argument("--world", default=WORLD)
    ap.add_argument("--route", default=json.dumps(ROUTE), help="JSON [[x,y],...]")
    a = ap.parse_args()
    label, params, reps = a.label, json.loads(a.params), a.reps
    route = [tuple(p) for p in json.loads(a.route)]
    WALLS.extend(load_walls(os.path.expanduser(a.world)))
    rclpy.init()
    n = RouteTester()
    n.ac.wait_for_server(timeout_sec=10)
    print(label, "set", params, n.set_params(n.ctrl_cli, {"FollowPath." + k: v for k, v in params.items()}), flush=True)
    legs, fails, alltr = [], [], []
    aborted = False
    for rep in range(reps):
        for gx, gy in route:
            r = n.go(gx, gy)
            legs.append(r)
            alltr.append(dict(r=r, trace=n.trace))
            if r["status"] != "OK":
                fails.append(dict(rep=rep, r=r, trace=n.trace))
                print("  FAIL", r, flush=True)
                if not n.recover():
                    print("  !! 回原点失败，中止")
                    aborted = True
                break
        if aborted:
            break
        print(f'  rep{rep}: legs so far {sum(l["status"] == "OK" for l in legs)}/{len(legs)}', flush=True)
    json.dump(fails, open(label + "_fails.json", "w"))
    json.dump(alltr, open(label + "_all.json", "w"))
    ok = [l for l in legs if l["status"] == "OK"]
    print(f'== {label}: 到达 {len(ok)}/{len(legs)}  成功段最大偏离 {max([l["xt_max"] for l in ok] or [0])} cm  '
          f'成功段p95偏离均值 {sum(l["xt_p95"] for l in ok) / max(1, len(ok)):.1f} cm  '
          f'全程车身最近 {min(l["body_min"] for l in legs)} cm  起点撞墙报错 {sum(l["lethal"] for l in legs)}  '
          f'均用时/段 {sum(l["t"] for l in ok) / max(1, len(ok)):.1f}s', flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
