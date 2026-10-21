#!/usr/bin/env python3
"""全局规划参数批量对比（只规划，车不动）。

对每组 global_costmap 参数：热改 → 等代价地图重新膨胀 → 对每个目标点调 /compute_path_to_pose，
统计成功数、路线总长、总转角、中心离墙。测完把改过的参数恢复成运行前的值。
  - 离墙（地图）：路线点到 /map 占据格中心的最近距离，仿真/实机都能用
  - 离墙（真值）：给了 --world 时，按 sdf 真实墙面算（只有仿真有）
只改 global_costmap 的参数（footprint_padding、inflation_layer.*）。SmacPlanner2D 的参数
在 Humble 下 ros2 param set 显示成功但不生效，要改 yaml 重启导航。

用法（导航已起来）：
  ros2 run my_bot_nav plan_sweep.py \
    --sets '[{"inflation_layer.inflation_radius":0.20},{"inflation_layer.inflation_radius":0.22}]'
  ros2 run my_bot_nav plan_sweep.py --sets '[{}]' --goals '[[1.6,0.8],[0.0,-1.75]]' --start 0,0.27
  ros2 run my_bot_nav plan_sweep.py --sets '[{}]' --world ~/dev_ws/src/my_bot/worlds/my_world_40.sdf
  ros2 run my_bot_nav plan_sweep.py --sets '[{}]' --goals '[[1.75,0.0]]' --save-plans /tmp/plans   # 存路线给离线仿真
--sets 里每组是 {参数名: 值}，{} = 用当前值；--start 不给就从机器人当前位置规划。
"""
import argparse
import json
import math
import os
import re
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

COSTMAP = "/global_costmap/global_costmap"
# my_world_40 的覆盖点：两个 40cm 拐角、两个死胡同、顶部、右上斜口、最底一排、左下
DEFAULT_GOALS = [[1.6, -0.6], [-1.6, -0.6], [1.65, 0.75], [-1.65, 0.75],
                 [0.0, 1.55], [1.45, 1.6], [0.0, -1.75], [-1.5, -1.25]]


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


def wall_dist(x, y, walls):
    return min(math.hypot(max(a - x, 0, x - b), max(c - y, 0, y - d)) for a, b, c, d in walls)


def path_metrics(p):
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))
    # 按 0.2m 重采样再累计转角，避开 5cm 栅格的锯齿
    pts, acc = [p[0]], 0.0
    for q in p[1:]:
        acc += math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1])
        if acc >= 0.2:
            pts.append(q)
            acc = 0.0
    hd = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    turn = sum(abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi) for h1, h2 in zip(hd, hd[1:]))
    return length, math.degrees(turn)


class Sweeper(Node):
    def __init__(self):
        super().__init__("plan_sweep")
        self.set_cli = self.create_client(SetParameters, COSTMAP + "/set_parameters")
        self.get_cli = self.create_client(GetParameters, COSTMAP + "/get_parameters")
        self.ac = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.occ = None

    def call(self, cli, req):
        cli.wait_for_service(timeout_sec=10)
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        return f.result()

    def get_params(self, names):
        req = GetParameters.Request()
        req.names = names
        return {n: parameter_value_to_python(v) for n, v in zip(names, self.call(self.get_cli, req).values)}

    def set_params(self, params):
        if not params:
            return True
        req = SetParameters.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]
        return all(r.successful for r in self.call(self.set_cli, req).results)

    def load_map(self):
        """读 /map 的占据格中心。先按 transient_local 收，收不到再按 volatile 收。"""
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            got = []
            qos = QoSProfile(depth=1, durability=dur, reliability=ReliabilityPolicy.RELIABLE)
            sub = self.create_subscription(OccupancyGrid, "/map", got.append, qos)
            t0 = time.time()
            while not got and time.time() - t0 < 5:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.destroy_subscription(sub)
            if got:
                m = got[0]
                r, w = m.info.resolution, m.info.width
                ox, oy = m.info.origin.position.x, m.info.origin.position.y
                self.occ = [(ox + (i % w + 0.5) * r, oy + (i // w + 0.5) * r)
                            for i, v in enumerate(m.data) if v >= 65]
                return True
        return False

    def map_dist(self, x, y):
        return min(math.hypot(x - a, y - b) for a, b in self.occ) if self.occ else float("nan")

    def plan(self, goal, start=None):
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = float(goal[0]), float(goal[1])
        g.goal.pose.orientation.w = 1.0
        if start:
            g.use_start = True
            g.start.header.frame_id = "map"
            g.start.pose.position.x, g.start.pose.position.y = start
            g.start.pose.orientation.w = 1.0
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=15)
        r = rf.result().result
        return ([(p.pose.position.x, p.pose.position.y) for p in r.path.poses],
                r.planning_time.sec + r.planning_time.nanosec * 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sets", required=True, help="JSON 列表，每组 {global_costmap 参数名: 值}")
    ap.add_argument("--goals", default=json.dumps(DEFAULT_GOALS), help="JSON [[x,y],...]，默认 my_world_40 的 8 个覆盖点")
    ap.add_argument("--start", default="", help="x,y；不给就从机器人当前位置规划")
    ap.add_argument("--world", default="", help="sdf 路径；给了就额外按真实墙面算离墙（仅仿真）")
    ap.add_argument("--wait", type=float, default=3.0, help="改参数后等代价地图重新膨胀的秒数")
    ap.add_argument("--out", default="", help="把每个点的结果写到这个 JSON 文件")
    ap.add_argument("--save-plans", default="", help="目录；把每条路线存成 JSON（离线仿真 neupan_sweep.py 的参考路线）")
    a = ap.parse_args()
    sets, goals = json.loads(a.sets), json.loads(a.goals)
    start = tuple(map(float, a.start.split(","))) if a.start else None
    walls = load_walls(a.world) if a.world else None

    rclpy.init()
    n = Sweeper()
    n.ac.wait_for_server(timeout_sec=10)
    print("地图占据格:", "已读取" if n.load_map() else "读不到 /map，离墙(地图)记 nan", flush=True)
    keys = sorted({k for s in sets for k in s})
    orig = n.get_params(keys) if keys else {}
    print("运行前的值:", orig, flush=True)
    out = []
    if a.save_plans:
        os.makedirs(a.save_plans, exist_ok=True)
    try:
        for si, s in enumerate(sets):
            ok = n.set_params(s)
            time.sleep(a.wait)
            rows = []
            for gx, gy in goals:
                p, pt = n.plan((gx, gy), start)
                if a.save_plans and p:
                    f = f"s{si}_{p[0][0]:.2f}_{p[0][1]:.2f}__{gx:.2f}_{gy:.2f}.json"
                    json.dump(dict(set=s, start=p[0], goal=(gx, gy), path=p), open(os.path.join(a.save_plans, f), "w"))
                row = dict(goal=(gx, gy), ok=bool(p), plan_ms=round(pt * 1000, 1))
                if p:
                    row["len"], row["turn"] = path_metrics(p)
                    row["map_clr"] = min(n.map_dist(x, y) for x, y in p)
                    if walls:
                        row["wall_clr"] = min(wall_dist(x, y, walls) for x, y in p)
                rows.append(row)
            good = [r for r in rows if r["ok"]]
            line = (f"{json.dumps(s)} set={ok}: 成功 {len(good)}/{len(rows)}"
                    f"  总长 {sum(r['len'] for r in good):.2f}m  总转角 {sum(r['turn'] for r in good):.0f}°"
                    f"  中心离墙(地图格中心) {min([r['map_clr'] for r in good] or [float('nan')]):.3f}m")
            if walls:
                line += f"  中心离墙(真实墙面) {min([r['wall_clr'] for r in good] or [float('nan')]):.3f}m"
            line += f"  最长规划 {max([r['plan_ms'] for r in rows] or [0]):.0f}ms"
            fails = [r["goal"] for r in rows if not r["ok"]]
            if fails:
                line += f"  失败点 {fails}"
            print(line, flush=True)
            out.append(dict(set=s, rows=rows))
    finally:
        n.set_params(orig)
        time.sleep(1)
        print("已恢复:", n.get_params(keys) if keys else "（没改参数）", flush=True)
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
