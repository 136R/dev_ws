#!/usr/bin/env python3
"""全局路线有多弯（只规划，车不动）—— 估 RPP 弯道减速后一圈要多久。

对路线每一段（相邻两点）调 /compute_path_to_pose（指定起点），按 5cm 重采样后用 0.15m 弦长
（≈ RPP 前视）算转弯半径，统计：
  - R<0.3 / R<0.45 / R<0.9 的路线长度、左右换向次数
  - 估时：每 5cm 按 RPP 弯道减速规则 v = vel × min(1, R / min_radius)（不低于 min_speed）累计。
    只算弯道减速，不含原地转、终点减速，所以比实测一圈短（my_world_40 上 ~90s 对实测 ~120s），只用来组间比
  - 车身离墙：给了 --world 时，车头取路线切向，按真实车身外廓算到 sdf 墙面的最近距离（只有仿真有）
SmacPlanner2D 的参数热改不生效，比较不同 w_smooth 等要改 yaml（或用 params_file:= 指副本）重启导航再跑。

用法（导航已起来）：
  ros2 run my_bot_nav plan_curvature.py --world ~/dev_ws/src/my_bot/worlds/my_world_40.sdf
  ros2 run my_bot_nav plan_curvature.py --route '[[0,0],[1.6,-0.6],[0,0]]' --min-radius 0.9 --out paths.json
--min-radius / --vel / --min-speed 对应 RPP 的 regulated_linear_scaling_min_radius / desired_linear_vel /
regulated_linear_scaling_min_speed。默认路线 = nav_route_test.py 的默认路线（从原点出发）。
"""
import argparse
import json
import math
import os
import re

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

ROUTE = [(0.0, 0.0), (0.99, -1.71), (0.0, 0.0), (1.6, -0.6), (0.0, 1.55), (-1.65, 0.75), (-1.5, -1.25), (0.0, 0.0)]
# 裸车身外廓（base_link 系），与 nav_route_test.py 同一份
BODY = [[0.063, 0.0615], [0.035, 0.11], [-0.0765, 0.11], [-0.1275, 0.0805], [-0.157, 0.0295],
        [-0.157, -0.0295], [-0.1275, -0.0805], [-0.0765, -0.11], [0.035, -0.11], [0.063, -0.0615]]
STEP = 0.05  # 重采样间距
CHORD = 3    # 外接圆取前后各 CHORD 个点，即前后各 0.15m


def load_walls(path):
    """sdf 墙体 → 轴对齐矩形 (xmin, xmax, ymin, ymax)。只处理外墙 + 0/90° 的 obstacle_* 墙。"""
    s = open(os.path.expanduser(path)).read()
    walls = [(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)
             for x, y, sx, sy in [(2, 0, 0.1, 4), (0, 2, 4, 0.1), (-2, 0, 0.1, 4), (0, -2, 4, 0.1)]]
    for m in re.finditer(r'<link name="(obstacle_[^"]+)">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', s, re.S):
        x, y, _, _, _, yaw = map(float, m.group(2).split())
        sx, sy, _ = map(float, m.group(3).split())
        if abs(abs(yaw) - 1.5708) < 0.01:
            sx, sy = sy, sx
        walls.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return walls


def sdist(x, y, walls):
    """点到最近墙面的有符号距离（在墙内为负）。"""
    best = 9.0
    for a, b, c, d in walls:
        dx, dy = max(a - x, 0, x - b), max(c - y, 0, y - d)
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


def body_min(p, walls):
    """路线上车身离墙最小值，车头取路线切向。"""
    best = 9.0
    for a, b in zip(p, p[1:]):
        th = math.atan2(b[1] - a[1], b[0] - a[0])
        c, s = math.cos(th), math.sin(th)
        best = min(best, min(sdist(b[0] + c * px - s * py, b[1] + s * px + c * py, walls) for px, py in SAMP))
    return best


def resample(p):
    out, acc = [p[0]], 0.0
    for a, b in zip(p, p[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        acc += seg
        while acc >= STEP:
            t = 1 - (acc - STEP) / seg if seg > 0 else 1
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            acc -= STEP
    return out


def radii(p):
    """每点用前后各 CHORD 个点的外接圆算有符号转弯半径（左正右负，直线记 99）。"""
    r = []
    for i in range(CHORD, len(p) - CHORD):
        a, b, c = p[i - CHORD], p[i], p[i + CHORD]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        R = math.dist(a, b) * math.dist(b, c) * math.dist(a, c) / (2 * abs(cross)) if abs(cross) > 1e-9 else 99
        r.append(math.copysign(R, cross))
    return r


def plan(node, ac, start, goal):
    g = ComputePathToPose.Goal()
    g.use_start = True
    g.planner_id = "GridBased"
    for ps, (x, y) in ((g.start, start), (g.goal, goal)):
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.w = 1.0
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    rf = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=15)
    return [(q.pose.position.x, q.pose.position.y) for q in rf.result().result.path.poses]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default=json.dumps(ROUTE), help="JSON [[x,y],...]，相邻两点为一段")
    ap.add_argument("--min-radius", type=float, default=0.9, help="RPP regulated_linear_scaling_min_radius")
    ap.add_argument("--vel", type=float, default=0.4, help="RPP desired_linear_vel")
    ap.add_argument("--min-speed", type=float, default=0.1, help="RPP regulated_linear_scaling_min_speed")
    ap.add_argument("--world", default="", help="sdf 路径；给了就按真实车身算离墙（仅仿真）")
    ap.add_argument("--out", default="", help="把每段原始路线写到这个 JSON 文件")
    a = ap.parse_args()
    route = [tuple(p) for p in json.loads(a.route)]
    walls = load_walls(a.world) if a.world else None

    rclpy.init()
    node = rclpy.create_node("plan_curvature")
    ac = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    ac.wait_for_server(timeout_sec=10)
    tot = dict(L=0.0, r3=0.0, r45=0.0, r9=0.0, flips=0, t=0.0)
    bmin, fails, paths = 9.0, 0, []
    for s, g in zip(route, route[1:]):
        raw = plan(node, ac, s, g)
        paths.append(raw)
        if len(raw) < 2:
            print(f"{s}->{g}: 规划失败", flush=True)
            fails += 1
            continue
        p = resample(raw)
        r = radii(p)
        seg = dict(L=STEP * len(p),
                   r3=STEP * sum(abs(R) < 0.3 for R in r),
                   r45=STEP * sum(abs(R) < 0.45 for R in r),
                   r9=STEP * sum(abs(R) < a.min_radius for R in r),
                   flips=sum(abs(x) < 2 and abs(y) < 2 and (x > 0) != (y > 0) for x, y in zip(r, r[1:])),
                   t=sum(STEP / max(a.min_speed, a.vel * min(1, abs(R) / a.min_radius)) for R in r))
        line = (f"{s}->{g}: 长 {seg['L']:.2f}m  R<0.3 {seg['r3']:.2f}m  R<0.45 {seg['r45']:.2f}m  "
                f"R<{a.min_radius} {seg['r9']:.2f}m  左右换向 {seg['flips']}  估时 {seg['t']:.1f}s")
        if walls:
            bm = body_min(p, walls)
            bmin = min(bmin, bm)
            line += f"  车身离墙 {bm * 100:.1f}cm"
        print(line, flush=True)
        for k in tot:
            tot[k] += seg[k]
    line = (f"== 合计: 长 {tot['L']:.2f}m  R<0.3 {tot['r3']:.2f}m  R<0.45 {tot['r45']:.2f}m  "
            f"R<{a.min_radius} {tot['r9']:.2f}m ({100 * tot['r9'] / max(tot['L'], 1e-6):.0f}%)  "
            f"左右换向 {tot['flips']}  估时 {tot['t']:.1f}s")
    if walls and bmin < 9.0:
        line += f"  车身离墙min {bmin * 100:.1f}cm"
    if fails:
        line += f"  规划失败 {fails} 段"
    print(line, flush=True)
    if a.out:
        json.dump(paths, open(a.out, "w"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
