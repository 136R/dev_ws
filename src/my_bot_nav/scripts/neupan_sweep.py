#!/usr/bin/env python3
"""NeuPAN 参数离线批量对比（车不动、不开 Gazebo）。

把 sdf 墙面 + plan_sweep.py --save-plans 存下的 SmacPlanner2D 路线拼成场景，对每组参数调
neupan_offline_sim（libneupan 闭环仿真：RotationShim → NeuPAN → velocity_smoother → 底盘限速限加速度 → 运动学），
多核并行，按 到达数 → 撞墙 → 车身离墙 → 左右摆次数 → 用时 排序。筛出前几组再回 Gazebo 验收。

用法：
  ros2 run my_bot_nav plan_sweep.py --sets '[{}]' --start 0,0 --goals '[[1.75,0.0]]' --save-plans /tmp/plans   # 先存路线（导航要起着）
  ros2 run my_bot_nav neupan_sweep.py --world <sdf> --plans /tmp/plans \\
      --route '[[1.75,0.0],[0.0,0.0],[-1.3,1.45],[0.0,0.0],[-0.3,-1.73],[0.0,0.0]]' \\
      --grid '{"adjust.q_s":[[1,1,0.2],[2.5,2.5,0.2]],"adjust.eta":[10,15]}' --out /tmp/sweep
  --sets '[{...},{...}]' 代替 --grid 可逐组指定。键：neupan yaml 的点号路径；"sim.xxx" 改仿真设置
  （如 sim.grid_res=0.05、sim.stride=2 模拟旧代价地图）；"model" 指定 DUNE 模型（dune_exact 时不用）；
  值写 null 表示删掉该键（例：{"robot.vertices": null, "robot.length": 0.22, ...} 换回矩形）。
--route 与 nav_route_test 相同：从原点出发、首尾相接；每段都要有存好的路线。
"""
import argparse
import concurrent.futures as cf
import copy
import itertools
import json
import math
import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from plan_sweep import load_walls  # noqa: E402

SHARE = get_package_share_directory("my_bot_nav")
# libneupan 是纯 CMake 包（不在 ament 索引里），按工作空间 install 目录找：与 my_bot_nav 同级
SIM_BIN = os.path.join(os.path.dirname(get_package_prefix("my_bot_nav")), "libneupan", "lib", "libneupan", "neupan_offline_sim")
# 裸车身外廓（base_link 系），与 nav_route_test.py / laser_filters.yaml 的 body_polygon_filter 同一份
BODY = [[0.063, 0.0615], [0.035, 0.11], [-0.0765, 0.11], [-0.1275, 0.0805], [-0.157, 0.0295],
        [-0.157, -0.0295], [-0.1275, -0.0805], [-0.0765, -0.11], [0.035, -0.11], [0.063, -0.0615]]


def sim_settings(nav2_yaml, base_yaml):
    """从导航参数和底盘控制器参数里取仿真要模拟的数值，保证和实际运行的一致。"""
    n, b = yaml.safe_load(open(nav2_yaml)), yaml.safe_load(open(base_yaml))
    cs = n["controller_server"]["ros__parameters"]
    fp, lc = cs["FollowPath"], n["local_costmap"]["local_costmap"]["ros__parameters"]
    vs = n["velocity_smoother"]["ros__parameters"]
    dc = b["diff_cont"]["ros__parameters"]
    return dict(
        ctrl_hz=cs["controller_frequency"], dt=1.0 / b["controller_manager"]["ros__parameters"]["update_rate"],
        grid_res=lc["resolution"], window=float(lc["width"]), stride=fp["obstacle_stride"], jitter=0.005,
        smoother_max_vel=[vs["max_velocity"][0], vs["max_velocity"][2]], smoother_acc=[vs["max_accel"][0], vs["max_accel"][2]],
        base_max_vel=[dc["linear"]["x"]["max_velocity"], dc["angular"]["z"]["max_velocity"]],
        base_acc=[dc["linear"]["x"]["max_acceleration"], dc["angular"]["z"]["max_acceleration"]],
        shim_threshold=fp["angular_dist_threshold"], shim_forward_dist=fp["forward_sampling_distance"],
        shim_vel=fp["rotate_to_heading_angular_vel"], shim_acc=fp["max_angular_accel"],
        goal_tol=cs["general_goal_checker"]["xy_goal_tolerance"],
        progress_radius=cs["progress_checker"]["required_movement_radius"],
        progress_time=cs["progress_checker"]["movement_time_allowance"],
        timeout=60.0, collision_tol=0.005,
        # raycast：按激光逐束打墙重建局部代价地图（标记 + 沿射线清除 + 脚印清除），与 Nav2 ObstacleLayer 一致
        costmap_mode="raycast", lidar_rate=10.0, lidar_beams=720, lidar_range=[0.1, 8.0], lidar_noise=0.005,
        lidar_offset=[-0.047, 0.0],  # robot.urdf.xacro 的 lidar_x/lidar_y；量程/束数/噪声见 lidar.xacro
        obstacle_max_range=lc["obstacle_layer"]["scan"]["obstacle_max_range"],
        raytrace_max_range=lc["obstacle_layer"]["scan"]["raytrace_max_range"],
        footprint=pad_footprint(json.loads(lc["footprint"]), lc.get("footprint_padding", 0.01)))


def pad_footprint(fp, pad):
    """同 nav2_costmap_2d::padFootprint：每个顶点按坐标符号往外推 pad。"""
    return [[x + math.copysign(pad, x), y + math.copysign(pad, y)] for x, y in fp]


def load_plans(d):
    plans = []
    for f in sorted(os.listdir(d)):
        if f.endswith(".json") and f.startswith("s"):
            p = json.load(open(os.path.join(d, f)))
            if "path" in p:
                plans.append(p)
    return plans


def pick_plan(plans, a, b):
    """目标与 b 一致、起点离 a 最近的那条路线。"""
    cand = [p for p in plans if math.hypot(p["goal"][0] - b[0], p["goal"][1] - b[1]) < 1e-3]
    if not cand:
        sys.exit(f"没有 {a}→{b} 的路线，先用 plan_sweep.py --start {a[0]},{a[1]} --goals '[[{b[0]},{b[1]}]]' --save-plans 存")
    best = min(cand, key=lambda p: math.hypot(p["start"][0] - a[0], p["start"][1] - a[1]))
    if math.hypot(best["start"][0] - a[0], best["start"][1] - a[1]) > 0.1:
        sys.exit(f"{a}→{b} 的路线起点 {best['start']} 离 {a} 超过 10cm")
    return best["path"]


def build_scenario(walls, route, plans, sim):
    legs, a = [], (0.0, 0.0)
    for i, b in enumerate(route):
        path = pick_plan(plans, a, b)
        th = 0.0 if i == 0 else math.atan2(path[min(5, len(path) - 1)][1] - path[0][1], path[min(5, len(path) - 1)][0] - path[0][0])
        legs.append(dict(name=f"{a[0]:.2f},{a[1]:.2f}->{b[0]:.2f},{b[1]:.2f}", start=[a[0], a[1], th], goal=list(b), path=path))
        a = tuple(b)
    return dict(walls=[list(w) for w in walls], body=BODY, legs=legs, sim=sim)


def put(tree, key, val):
    """点号路径赋值；val 为 null 时删掉这个键（如换回矩形车体要删 robot.vertices）。"""
    ks = key.split(".")
    for k in ks[:-1]:
        tree = tree.setdefault(k, {})
    if val is None:
        tree.pop(ks[-1], None)
    else:
        tree[ks[-1]] = val


def run_one(args):
    idx, overrides, base_planner, scenario, outdir, default_model = args
    d = os.path.join(outdir, f"g{idx:04d}")
    os.makedirs(d, exist_ok=True)
    planner, sc, model = copy.deepcopy(base_planner), copy.deepcopy(scenario), default_model
    for k, v in overrides.items():
        if k == "model":
            model = v
        elif k.startswith("sim."):
            sc["sim"][k[4:]] = v
        else:
            put(planner, k, v)
    yaml.safe_dump(planner, open(os.path.join(d, "planner.yaml"), "w"), sort_keys=False)
    json.dump(sc, open(os.path.join(d, "scenario.json"), "w"))
    cmd = [SIM_BIN, os.path.join(d, "scenario.json"), os.path.join(d, "planner.yaml")] + ([model] if model else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
    legs = [json.loads(l) for l in r.stdout.splitlines() if l.startswith("{")]
    res = dict(idx=idx, set=overrides, legs=legs, err=r.stderr.strip()[-300:] if r.returncode else "")
    json.dump(res, open(os.path.join(d, "result.json"), "w"), ensure_ascii=False)
    return res


def summarize(r):
    L = r["legs"]
    ok = [l for l in L if l["status"] == "OK"]
    return dict(ok=len(ok), n=len(L), col=sum(l["status"] == "COLLISION" for l in L),
                stuck=sum(l["status"] in ("STUCK", "TIMEOUT") for l in L),
                body=min([l["body_min"] for l in L] or [float("nan")]),
                flips=sum(l["flips"] for l in L), stops=sum(l["stops"] for l in L),
                t=sum(l["t"] for l in ok) / max(1, len(ok)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--plans", required=True, help="plan_sweep.py --save-plans 的目录")
    ap.add_argument("--route", required=True, help="JSON [[x,y],...]，从原点出发")
    ap.add_argument("--grid", default="", help='JSON {键: [取值...]}，做笛卡尔积')
    ap.add_argument("--sets", default="", help="JSON [{键: 值}, ...]，逐组指定（和 --grid 二选一）")
    ap.add_argument("--planner", default=os.path.join(SHARE, "config/sim/neupan_sim.yaml"))
    ap.add_argument("--nav2", default=os.path.join(SHARE, "config/sim/nav2_params_neupan_smac2d.yaml"))
    ap.add_argument("--base", default=os.path.join(get_package_share_directory("my_bot"), "config/my_controllers.yaml"))
    ap.add_argument("--model", default="", help="DUNE 模型；planner 开了 dune_exact 时不需要")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--out", required=True)
    ap.add_argument("--top", type=int, default=15)
    a = ap.parse_args()

    if a.grid:
        g = json.loads(a.grid)
        sets = [dict(zip(g, vals)) for vals in itertools.product(*g.values())]
    else:
        sets = json.loads(a.sets or "[{}]")
    walls = load_walls(a.world)
    scenario = build_scenario(walls, [tuple(p) for p in json.loads(a.route)], load_plans(a.plans), sim_settings(a.nav2, a.base))
    base_planner = yaml.safe_load(open(a.planner))
    if not a.model:  # 和导航一样用 nav2 参数里的 dune_checkpoint（dune_exact 时只是占位，不读）
        ck = yaml.safe_load(open(a.nav2))["controller_server"]["ros__parameters"]["FollowPath"]["dune_checkpoint"]
        a.model = ck if os.path.isabs(ck) else os.path.join(SHARE, ck)
    os.makedirs(a.out, exist_ok=True)
    print(f"{len(sets)} 组 × {len(scenario['legs'])} 段，{a.jobs} 并行；仿真设置 {scenario['sim']}", flush=True)

    results = []
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        futs = [ex.submit(run_one, (i, s, base_planner, scenario, a.out, a.model)) for i, s in enumerate(sets)]
        for k, f in enumerate(cf.as_completed(futs), 1):
            results.append(f.result())
            if k % max(1, len(sets) // 10) == 0:
                print(f"  {k}/{len(sets)}", flush=True)
    errs = [r for r in results if r["err"] or not r["legs"]]
    for r in errs[:3]:
        print("!! 出错", r["set"], r["err"], flush=True)

    rows = [(summarize(r), r) for r in results if r["legs"]]
    rows.sort(key=lambda x: (-x[0]["ok"], x[0]["col"], -min(x[0]["body"], 3.0), x[0]["flips"], x[0]["t"]))
    json.dump([dict(summary=s, **r) for s, r in rows], open(os.path.join(a.out, "summary.json"), "w"), ensure_ascii=False, indent=1)
    print(f"\n{'到达':>6} {'撞墙':>4} {'卡住':>4} {'离墙cm':>7} {'摆':>4} {'急停':>4} {'s/段':>6}  参数")
    for s, r in rows[:a.top]:
        print(f"{s['ok']:>3}/{s['n']:<2} {s['col']:>4} {s['stuck']:>4} {s['body']:>7.1f} {s['flips']:>4} {s['stops']:>4} {s['t']:>6.1f}  {json.dumps(r['set'])}")
    print(f"\n全部结果：{os.path.join(a.out, 'summary.json')}")


if __name__ == "__main__":
    main()
