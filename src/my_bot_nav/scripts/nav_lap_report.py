#!/usr/bin/env python3
"""仿真跑圈验收报告：跑 nav_route_test 若干圈，同时旁听 Gazebo 真值和导航日志，打印一张能直接看懂的表。

用你自己起好的仿真 + 定位 + 导航（车停在原点）。改了 NeuPAN 参数要重启导航才生效（yaml 只在启动时读）。
  ros2 run my_bot_nav nav_lap_report.py run  F3c --laps 3                 # my_world_40 默认 7 段路线
  ros2 run my_bot_nav nav_lap_report.py run  F3c_u --laps 3 --map unit    # nav_unit_40 测试场 A/B/C
  ros2 run my_bot_nav nav_lap_report.py show F3c P6 F3c_u                 # 已存的几次结果并排比
结果存在 ~/nav_reports/<标签>.json（含当时的 NeuPAN 关键参数），show 时按标签读。

批量跑多组参数（脚本自己起停 仿真+定位+导航，开跑前你那套必须关掉）：
  ros2 run my_bot_nav nav_lap_report.py batch groups.yaml
groups.yaml（顶层是默认值，每组可覆盖 laps/map/stuck；没写的参数保持原值）：
  laps: 3
  map: unit                  # maze / unit
  stuck: 15                  # 秒；这么久挪不到 3cm、转不到 10° 算卡死
  controller: neupan_smac2d  # nav.launch.py 的 controller:=
  groups:
    - label: A
      neupan:                # NeuPAN yaml 点号路径，写进临时 yaml，导航生命周期重置后重读
        adjust.eta: 10
      nav2:                  # 节点名: {参数: 值}，运行中直接改
        global_costmap/global_costmap: {footprint_padding: 0.02}
某段卡死/失败 → 脱困开回原点，跳过这段接着跑；开回原点也卡死 → 重启整套再接着跑下一段；
同一组重启满 2 次记为中止，换下一组。全部跑完并排出表；仿真/定位/导航日志在 ~/nav_reports/batch_<时间>/。

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
import signal
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
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import nav_route_test as nrt  # noqa: E402
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
        self.toff, self.t_wall = 0.0, 0.0
        self.create_subscription(PoseStamped, "/gz/model/my_bot/pose", self.truth_cb, 50)
        self.create_subscription(Log, "/rosout", self.log_cb, 200)

    def truth_cb(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + self.toff
        if self.truth and t < self.truth[-1][0] - 1.0:  # 仿真重启后时间从 0 开始，接到上一段后面
            self.toff += self.truth[-1][0] - t
            t = self.truth[-1][0]
        self.truth.append((t, m.pose.position.x, m.pose.position.y, yaw))
        self.t_wall = time.time()

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

    def call(self, srv, name, req, timeout=10.0):
        """同步调服务（本节点在后台线程 spin）；超时返回 None。"""
        cli = self.create_client(srv, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        f = cli.call_async(req)
        t0 = time.time()
        while not f.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return f.result() if f.done() else None


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
        ["跳过段 / 重启"] + [f"{x['legs'] - x['legs_ok']} / {r.get('restarts', '—')}" + (" 中止" if r.get("aborted") else "")
                             for x, r in zip(m, results)],
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
        for l in r.get("legs", []):
            if l.get("from_origin"):
                print(f"  {r['label']} 起点不同（上一段跳过，从原点出发）：去 {l['goal']} {l['status']}")


# ---------- batch：脚本自己起停整套，按参数组依次跑 ----------
STACK_PAT = "gz sim|slam_toolbox|nav2_|bt_navigator|parameter_bridge|laser_filter|ekf_node|task_manager|ros_gui_backend"


class Stack:
    """脚本自己起的 仿真 + 定位 + 导航；每条 launch 单独一个进程组，关的时候只关这几组。"""

    def __init__(self, logdir, controller):
        self.logdir, self.controller = logdir, controller
        self.procs, self.n, self.map = [], 0, None

    def start(self, map_key):
        world_file = MAPS[map_key][0]
        name = os.path.splitext(world_file)[0]  # ~/.maps 下与世界同名的地图
        cmds = [["ros2", "launch", "my_bot", "launch_sim.launch.py", "world:=" + os.path.join(WORLDS, world_file)],
                ["ros2", "launch", "my_bot_slam", "slam.launch.py", "use_sim_time:=true", "mode:=localization", "map:=" + name],
                ["ros2", "launch", "my_bot_nav", "nav.launch.py", "use_sim_time:=true",
                 "controller:=" + self.controller, "map:=" + name]]
        self.n += 1
        self.map = map_key
        print(f"[启动 #{self.n}] {name}，日志 {self.logdir}/stack{self.n}_*.log", flush=True)
        for i, c in enumerate(cmds):
            log = open(os.path.join(self.logdir, f"stack{self.n}_{i}.log"), "w")
            self.procs.append(subprocess.Popen(c, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            time.sleep(5)

    def stop(self):
        for p in reversed(self.procs):
            for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
                t0 = time.time()
                gone = False
                while not gone and time.time() - t0 < wait:
                    p.poll()  # 收掉 launch 父进程，不然它的僵尸让进程组一直"还在"
                    try:
                        os.killpg(p.pid, 0)
                        time.sleep(0.2)
                    except ProcessLookupError:
                        gone = True
                if gone:
                    break
        self.procs = []


def wait_ready(node, tester, timeout=180):
    """等 仿真真值 + 导航生命周期激活 + 定位 TF 都就绪，返回 RouteTester（首次新建，之后清掉旧仿真的 TF）。"""
    t0 = time.time()
    while time.time() - node.t_wall > 1.0:
        if time.time() - t0 > timeout:
            raise RuntimeError("收不到 Gazebo 真值，看 stack*_0.log")
        time.sleep(0.5)
    while True:
        r = node.call(Trigger, "/lifecycle_manager_navigation/is_active", Trigger.Request(), timeout=5)
        if r and r.success:
            break
        if time.time() - t0 > timeout:
            raise RuntimeError("导航没激活，看 stack*_2.log")
        time.sleep(2)
    if tester is None:
        tester = nrt.RouteTester()
    else:
        tester.buf.clear()  # 仿真重启后时间从 0 开始，旧 TF 时间戳更大，不清会一直查到旧位姿
    while tester.pose(tries=20) is None:
        if time.time() - t0 > timeout:
            raise RuntimeError("查不到 map→base_footprint，看 stack*_1.log")
    tester.ac.wait_for_server(timeout_sec=30)
    return tester


def has_dotted(y, key):
    for k in key.split("."):
        if not isinstance(y, dict) or k not in y:
            return False
        y = y[k]
    return True


def set_dotted(y, key, v):
    ks = key.split(".")
    for k in ks[:-1]:
        y = y[k]
    y[ks[-1]] = v


def apply_group(node, g, base_cfg, workdir):
    """NeuPAN 参数写进临时 yaml、Nav2 参数直接设，再让导航生命周期 RESET+STARTUP，控制器重读 yaml。"""
    y = yaml.safe_load(open(base_cfg))
    for k, v in (g.get("neupan") or {}).items():
        set_dotted(y, k, v)
    cfg = os.path.join(workdir, g["label"] + "_neupan.yaml")
    with open(cfg, "w") as f:
        yaml.safe_dump(y, f, allow_unicode=True, sort_keys=False)
    sets = {n: dict(p) for n, p in (g.get("nav2") or {}).items()}
    sets.setdefault("controller_server", {})["FollowPath.config_file"] = cfg
    for n, p in sets.items():
        req = SetParameters.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in p.items()]
        r = node.call(SetParameters, f"/{n}/set_parameters", req)
        if r is None:
            raise ValueError(f"/{n}/set_parameters 连不上")
        bad = [f"{n} {k}: {x.reason}" for k, x in zip(p, r.results) if not x.successful]
        if bad:
            raise ValueError("参数设置失败 " + "; ".join(bad))
    for cmd in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
        req = ManageLifecycleNodes.Request()
        req.command = cmd
        r = node.call(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes", req, timeout=60)
        if not (r and r.success):
            raise RuntimeError("导航生命周期 RESET/STARTUP 失败，看 stack*_2.log")
    time.sleep(3)  # 让代价地图收齐地图和雷达


def batch(path):
    spec = yaml.safe_load(open(path))
    dflt = dict(laps=spec.get("laps", 3), map=spec.get("map", "maze"), stuck=spec.get("stuck", 15))
    groups = [dict(dflt, **g) for g in spec["groups"]]
    labels = [g["label"] for g in groups]
    if len(set(labels)) != len(labels):
        sys.exit(f"标签重复：{labels}")
    bad = [g["label"] for g in groups if g["map"] not in MAPS]
    if bad:
        sys.exit(f"这些组的 map 不是 {list(MAPS)}：{bad}")
    running = [l for l in subprocess.run(["pgrep", "-af", STACK_PAT], capture_output=True, text=True).stdout.splitlines()
               if "nav_lap_report" not in l]
    if running:
        sys.exit("已经有仿真/定位/导航在跑，先关掉再跑 batch：\n" + "\n".join(running))

    workdir = os.path.join(OUTDIR, "batch_" + time.strftime("%m%d_%H%M%S"))
    os.makedirs(workdir)
    stack = Stack(workdir, spec.get("controller", "neupan_smac2d"))
    rclpy.init()
    node = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    results, tester, base_cfg = [], None, None
    try:
        for g in groups:
            if stack.map != g["map"]:
                stack.stop()
                stack.start(g["map"])
                tester = wait_ready(node, tester)
            if base_cfg is None:  # 第一次起来时 controller_server 指着的 NeuPAN yaml，各组都在它上面改
                base_cfg = node.neupan_params()[1]
                y = yaml.safe_load(open(base_cfg))
                miss = sorted({k for gg in groups for k in (gg.get("neupan") or {}) if not has_dotted(y, k)})
                if miss:
                    sys.exit(f"{base_cfg} 里没有这些键（拼错了？）：{miss}")
            world = os.path.join(WORLDS, MAPS[g["map"]][0])
            walls = load_walls(world)
            nrt.WALLS[:] = nrt.load_walls(world)
            route = MAPS[g["map"]][1] or nrt.ROUTE
            print(f"\n=== {g['label']}：{g['map']} {g['laps']} 圈，卡死 {g['stuck']}s ===", flush=True)
            try:
                apply_group(node, g, base_cfg, workdir)
            except ValueError as e:
                print(f"  !! 跳过这组：{e}", flush=True)
                continue
            params, cfg = node.neupan_params()
            node.truth.clear()
            node.logs.clear()
            legs, restarts, aborted, from_origin = [], 0, False, False
            for rep in range(g["laps"]):
                for gx, gy in route:
                    if from_origin and math.hypot(gx, gy) < 1e-6:  # 跳过后车已在原点，这段"回原点"不算
                        from_origin = False
                        continue
                    r = tester.go(gx, gy, stuck=g["stuck"])
                    r["rep"] = rep
                    if from_origin:
                        r["from_origin"] = True
                    legs.append(r)
                    print(f"  第{rep + 1}圈 去 ({gx}, {gy})：{r['status']} {r['t']}s", flush=True)
                    if r["status"] == "OK":
                        from_origin = False
                        continue
                    from_origin = math.hypot(gx, gy) > 1e-6  # 下一段本该从这段终点出发，现在从原点出发
                    if tester.recover(g["stuck"]):
                        print("  已回原点，跳过这段", flush=True)
                        continue
                    restarts += 1
                    print(f"  !! 回不了原点，重启整套（本组第 {restarts} 次）", flush=True)
                    stack.stop()
                    stack.start(g["map"])
                    tester = wait_ready(node, tester)
                    if restarts >= 2:
                        print("  !! 本组重启满 2 次，中止，换下一组", flush=True)
                        aborted = True
                        break
                    apply_group(node, g, base_cfg, workdir)
                if aborted:
                    break
            res = dict(label=g["label"], map=g["map"], laps=g["laps"], time=time.strftime("%Y-%m-%d %H:%M"),
                       params=params, config_file=cfg, restarts=restarts, aborted=aborted,
                       metrics=analyse(list(node.truth), list(node.logs), legs, walls), legs=legs)
            json.dump(res, open(os.path.join(OUTDIR, g["label"] + ".json"), "w"), ensure_ascii=False, indent=1)
            results.append(res)
    finally:
        if tester:
            tester.destroy_node()
        stack.stop()
        ex.shutdown()
        rclpy.shutdown()
    print()
    if results:
        show(results)
    print(f"\n各组结果存在 {OUTDIR}/<标签>.json；仿真/定位/导航日志和各组 NeuPAN yaml 在 {workdir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("label")
    r.add_argument("--laps", type=int, default=3)
    r.add_argument("--map", choices=list(MAPS), default="maze")
    r.add_argument("--stuck", type=float, default=0.0, help="秒；卡住这么久就放弃本次（回原点后不再跑后面的圈）")
    s = sub.add_parser("show")
    s.add_argument("labels", nargs="+")
    b = sub.add_parser("batch")
    b.add_argument("groups", help="参数组 yaml，格式见上")
    a = ap.parse_args()
    os.makedirs(OUTDIR, exist_ok=True)

    if a.cmd == "show":
        show([json.load(open(os.path.join(OUTDIR, l + ".json"))) for l in a.labels])
        return
    if a.cmd == "batch":
        batch(a.groups)
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
    if a.stuck:
        cmd += ["--stuck", str(a.stuck)]
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
