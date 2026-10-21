#!/usr/bin/env python3
"""仿真跑圈验收报告：跑 nav_route_test 若干圈，同时旁听 Gazebo 真值和导航日志，打印一张能直接看懂的表。

用你自己起好的仿真 + 定位 + 导航（车停在原点）。改了 NeuPAN 参数要重启导航才生效（yaml 只在启动时读）。
  ros2 run my_bot_nav nav_lap_report.py run  F3c --laps 3                 # my_world_40 默认 7 段路线
  ros2 run my_bot_nav nav_lap_report.py run  F3c_u --laps 3 --map unit    # nav_unit_40 测试场 A/B/C
  ros2 run my_bot_nav nav_lap_report.py show F3c P6 F3c_u                 # 已存的几次结果并排比
结果存在 ~/nav_reports/<标签>.json（含当时的 NeuPAN 关键参数），show 时按标签读。

表里各项：
  车身离墙最近（真值）   按 Gazebo 真实位姿 + 真实车身外廓算到 sdf 墙面，负数 = 擦墙
  擦墙时长               真值离墙 < 0 的累计秒数
  NeuPAN 急停次数        日志"NeuPAN 停止"，间隔 2s 内的算同一次
  恢复动作 / 没进展      行为树原地转/后退/等待次数、"Failed to make progress"次数
  车头左右翻转           真实车头每 0.25s 采一次，转向方向反过来记一次（|角速度| > 0.15 rad/s 才算）；
                         拐弯本身也会翻转，只适合同一路线的两组之间比
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import unicodedata

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from plan_sweep import load_walls  # noqa: E402

WORLDS = os.path.join(get_package_share_directory("my_bot"), "worlds")
SHARE = get_package_share_directory("my_bot_nav")
MAPS = {
    "maze": ("my_world_40.sdf", [(1.15, -1.71), (0.0, 0.0), (1.6, -0.6), (0.0, 1.55), (-1.65, 0.75), (-1.5, -1.25), (0.0, 0.0)]),  # None = nav_route_test 默认 7 段
    "unit": ("nav_unit_40.sdf", [[1.75, 0.0], [0.0, 0.0], [-1.3, 1.45], [0.0, 0.0], [-0.3, -1.73], [0.0, 0.0]]),
}
OUTDIR = os.path.expanduser("~/nav_reports")
# 裸车身外廓（base_link 系），与 nav_route_test.py / laser_filters.yaml 的 body_polygon_filter 同一份
BODY = [[0.063, 0.0615], [0.035, 0.11], [-0.0765, 0.11], [-0.1275, 0.0805], [-0.157, 0.0295],
        [-0.157, -0.0295], [-0.1275, -0.0805], [-0.0765, -0.11], [0.035, -0.11], [0.063, -0.0615]]
SAMP = []
for _i in range(len(BODY)):
    _a, _b = BODY[_i], BODY[(_i + 1) % len(BODY)]
    _n = max(1, int(math.hypot(_b[0] - _a[0], _b[1] - _a[1]) / 0.01))
    SAMP += [(_a[0] + (_b[0] - _a[0]) * k / _n, _a[1] + (_b[1] - _a[1]) * k / _n) for k in range(_n)]
# 表里列出的 NeuPAN 参数（yaml 点号路径）
PARAM_KEYS = ["adjust.q_s", "adjust.p_u", "adjust.eta", "adjust.d_min", "adjust.d_max",
              "robot.max_speed", "robot.max_acce", "receding", "ref_speed", "collision_threshold",
              "pan.nrmp_max_num", "pan.dune_exact"]


class Recorder(Node):
    def __init__(self):
        super().__init__("nav_lap_report")
        self.truth, self.logs = [], []
        self.create_subscription(PoseStamped, "/gz/model/my_bot/pose", self.truth_cb, 50)
        self.create_subscription(Log, "/rosout", self.log_cb, 200)

    def truth_cb(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.truth.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x, m.pose.position.y, yaw))

    def log_cb(self, m):
        if m.name in ("controller_server", "behavior_server"):
            self.logs.append((time.time(), m.name, m.msg))

    def neupan_params(self):
        """从 controller_server 读 NeuPAN 配置文件路径，再从文件里取关键参数。"""
        cli = self.create_client(GetParameters, "/controller_server/get_parameters")
        if not cli.wait_for_service(timeout_sec=5):
            return {}, ""
        req = GetParameters.Request()
        req.names = ["FollowPath.config_file", "FollowPath.config_package"]
        f = cli.call_async(req)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            time.sleep(0.05)
        if not f.done():
            return {}, ""
        path, pkg = [parameter_value_to_python(v) for v in f.result().values]
        if path and not os.path.isabs(path):
            path = os.path.join(get_package_share_directory(pkg or "my_bot_nav"), path)
        try:
            y = yaml.safe_load(open(path))
        except Exception:
            return {}, path
        out = {}
        for k in PARAM_KEYS:
            v = y
            for kk in k.split("."):
                v = v.get(kk) if isinstance(v, dict) else None
            out[k] = v
        out["robot.shape"] = "多边形" if (y.get("robot") or {}).get("vertices") else "矩形"
        return out, path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clearance(x, y, th, walls):
    c, s = math.cos(th), math.sin(th)
    best = 9.0
    for px, py in SAMP:
        wx, wy = x + c * px - s * py, y + s * px + c * py
        for a, b, cc, d in walls:
            dx, dy = max(a - wx, 0, wx - b), max(cc - wy, 0, wy - d)
            best = min(best, -min(wx - a, b - wx, wy - cc, d - wy) if dx == 0 and dy == 0 else math.hypot(dx, dy))
    return best


def episodes(ts, gap=2.0):
    n, last = 0, None
    for t in ts:
        if last is None or t - last > gap:
            n += 1
        last = t
    return n


def analyse(truth, logs, legs, walls):
    tr = []  # 10Hz
    for s in truth:
        if not tr or s[0] - tr[-1][0] >= 0.1:
            tr.append(s)
    clr = [clearance(x, y, th, walls) for _, x, y, th in tr]
    flips, sgn, last = 0, 0, None
    for s in truth:
        if last is None or s[0] - last[0] >= 0.25:
            if last is not None:
                w = wrap(s[3] - last[3]) / (s[0] - last[0])
                if abs(w) > 0.15:
                    k = 1 if w > 0 else -1
                    if sgn and k != sgn:
                        flips += 1
                    sgn = k
            last = s
    ok = [l for l in legs if l["status"] == "OK"]
    return dict(
        legs_ok=len(ok), legs=len(legs),
        clear_min=round(min(clr) * 100, 1) if clr else None,
        scrape_s=round(sum(0.1 for c in clr if c < 0), 1),
        stops=episodes([t for t, n, m in logs if "NeuPAN 停止" in m]),
        recoveries=sum(1 for t, n, m in logs if n == "behavior_server" and m.startswith("Running")),
        no_progress=sum(1 for t, n, m in logs if "Failed to make progress" in m),
        lethal=sum(l.get("lethal", 0) for l in legs),
        xt_max=max([l["xt_max"] for l in ok] or [0]),
        t_avg=round(sum(l["t"] for l in ok) / max(1, len(ok)), 1),
        flips=flips,
        fails=[dict(goal=l["goal"], status=l["status"], end=l["end"]) for l in legs if l["status"] != "OK"],
    )


# ---------- 表格 ----------
def dw(s):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))


def pad(s, w):
    return str(s) + " " * (w - dw(s))


def table(head, rows):
    cols = [head] + rows
    ws = [max(dw(r[i]) for r in cols) + 2 for i in range(len(head))]
    line = lambda l, m, r: l + m.join("─" * w for w in ws) + r
    out = [line("┌", "┬", "┐"), "│" + "│".join(" " + pad(h, w - 1) for h, w in zip(head, ws)) + "│"]
    for r in rows:
        out.append(line("├", "┼", "┤"))
        out.append("│" + "│".join(" " + pad(c, w - 1) for c, w in zip(r, ws)) + "│")
    out.append(line("└", "┴", "┘"))
    return "\n".join(out)


def fmt(v):
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    if isinstance(v, float):
        return f"{v:g}"
    return "—" if v is None else str(v)


def show(results):
    labels = [r["label"] for r in results]
    m = [r["metrics"] for r in results]
    rows = [
        ["地图 / 圈数"] + [f"{r['map']} / {r['laps']}" for r in results],
        ["到达"] + [f"{x['legs_ok']}/{x['legs']}" for x in m],
        ["车身离墙最近（真值）"] + [f"{x['clear_min']} cm" for x in m],
        ["擦墙时长"] + [f"{x['scrape_s']} s" for x in m],
        ["NeuPAN 急停次数"] + [x["stops"] for x in m],
        ["恢复动作 / 没进展"] + [f"{x['recoveries']} / {x['no_progress']}" for x in m],
        ["起点撞墙报错"] + [x["lethal"] for x in m],
        ["离路线最大偏差"] + [f"{x['xt_max']} cm" for x in m],
        ["平均每段"] + [f"{x['t_avg']} s" for x in m],
        ["车头左右翻转"] + [x["flips"] for x in m],
    ]
    keys = [k for k in ["robot.shape"] + PARAM_KEYS if any(r["params"].get(k) is not None for r in results)]
    for k in keys:
        vals = [fmt(r["params"].get(k)) for r in results]
        mark = "  ←不同" if len(set(vals)) > 1 else ""
        rows.append([k.split(".")[-1] + mark] + vals)
    print(table(["项目"] + labels, rows))
    for r in results:
        for f in r["metrics"]["fails"]:
            print(f"  {r['label']} 失败段：去 {f['goal']} {f['status']}，停在 {f['end']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("label")
    r.add_argument("--laps", type=int, default=3)
    r.add_argument("--map", choices=list(MAPS), default="maze")
    s = sub.add_parser("show")
    s.add_argument("labels", nargs="+")
    a = ap.parse_args()
    os.makedirs(OUTDIR, exist_ok=True)

    if a.cmd == "show":
        show([json.load(open(os.path.join(OUTDIR, l + ".json"))) for l in a.labels])
        return

    world_file, route = MAPS[a.map]
    world = os.path.join(WORLDS, world_file)
    walls = load_walls(world)
    rclpy.init()
    node = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.time()
    while not node.truth and time.time() - t0 < 10:
        time.sleep(0.1)
    if not node.truth:
        sys.exit("收不到 Gazebo 真值 /gz/model/my_bot/pose：仿真没起来，或 ROS 环境变量没 export")
    _, x, y, _ = node.truth[-1]
    if math.hypot(x, y) > 0.05:
        sys.exit(f"车不在原点（真值 {x:.3f},{y:.3f}），先开回原点再跑")
    params, cfg = node.neupan_params()
    print(f"NeuPAN 参数文件：{cfg or '读不到（导航起着吗？）'}", flush=True)

    node.truth.clear()
    node.logs.clear()
    work = tempfile.mkdtemp(prefix="lap_")
    cmd = ["ros2", "run", "my_bot_nav", "nav_route_test.py", a.label, "{}", str(a.laps), "--world", world]
    if route:
        cmd += ["--route", json.dumps(route)]
    subprocess.run(cmd, cwd=work)
    time.sleep(1.0)
    try:
        legs = [x["r"] for x in json.load(open(os.path.join(work, a.label + "_all.json")))]
    except FileNotFoundError:
        sys.exit("nav_route_test 没跑完（看上面的输出）")
    truth, logs = list(node.truth), list(node.logs)
    ex.shutdown()
    rclpy.shutdown()

    res = dict(label=a.label, map=a.map, laps=a.laps, time=time.strftime("%Y-%m-%d %H:%M"),
               params=params, config_file=cfg, metrics=analyse(truth, logs, legs, walls), legs=legs)
    json.dump(res, open(os.path.join(OUTDIR, a.label + ".json"), "w"), ensure_ascii=False, indent=1)
    print()
    show([res])
    print(f"\n已存 {os.path.join(OUTDIR, a.label + '.json')}；和别的结果比：nav_lap_report.py show {a.label} <其它标签>")


if __name__ == "__main__":
    main()
