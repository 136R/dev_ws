#!/usr/bin/env python3
"""仿真定位误差测试：拿 Gazebo 真值逐帧对比里程计和 slam 定位。

真值 /gz/model/my_bot/pose（base_footprint 在世界系，50Hz）；地图从原点建、车在原点出生，map = world。
每个真值帧按它的时间戳去查同一时刻的 TF：
  - odom→base_footprint：里程计（EKF）。只看"转动过程中转了多少"和真值差多少（odom 本身会随时间漂）
  - map→base_footprint：定位。直接和真值比位置 / 角度
两种模式：
  spin   车停着时用 /cmd_vel_keyboard 原地转指定角度（方向交替），量转动中最大误差、停稳时误差、多久恢复
  watch  只旁听 N 秒（例如 nav_route_test 跑路线时并行开），量全程 / 转动中 / 静止时误差分布

用法（仿真 + slam 定位已起来；spin 模式车要停着、周围有转身空间）：
  ros2 run my_bot_nav loc_error_test.py base spin --angles '[90,180,360]' --reps 3
  ros2 run my_bot_nav loc_error_test.py base_route watch --duration 900 --stop-file /tmp/route_done
结果写到当前目录 <标签>_loc.json。恢复判据：角度 < --tol-deg 且位置 < --tol-cm，并一直保持到窗口结束。
"""
import argparse
import json
import math
import os
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def q2yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pct(v, p):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))]


class LocErr(Node):
    def __init__(self):
        super().__init__("loc_error_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.pending = deque()
        self.yaw_last, self.turn_acc = None, 0.0   # 控制原地转用：直接按收到的真值累计，不等 TF 配对
        self.samples = []   # (t, 真值 x, y, yaw, odom yaw, map x, y, yaw)
        self.create_subscription(PoseStamped, "/gz/model/my_bot/pose", self.truth_cb, 50)
        self.kb_pub = self.create_publisher(Twist, "/cmd_vel_keyboard", 10)

    def truth_cb(self, m):
        t = Time.from_msg(m.header.stamp)
        yaw = q2yaw(m.pose.orientation)
        if self.yaw_last is not None:
            self.turn_acc += wrap(yaw - self.yaw_last)
        self.yaw_last = yaw
        self.pending.append((t, m.pose.position.x, m.pose.position.y, yaw))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def process(self):
        """把真值帧和同一时刻的 TF 配对。等 TF 到齐（≥0.15s 前的帧）再查，查不到超过 1s 就丢。"""
        now = self.get_clock().now()
        while self.pending:
            t, x, y, yaw = self.pending[0]
            age = (now - t).nanoseconds * 1e-9
            if age < 0.15:
                return
            self.pending.popleft()
            try:
                o = self.buf.lookup_transform("odom", "base_footprint", t)
                mp = self.buf.lookup_transform("map", "base_footprint", t)
            except Exception:
                continue
            tr = mp.transform.translation
            self.samples.append((t.nanoseconds * 1e-9, x, y, yaw, q2yaw(o.transform.rotation),
                                 tr.x, tr.y, q2yaw(mp.transform.rotation)))

    def spin_for(self, sec, twist=None, stop_file=None):
        t_end = self.now_s() + sec
        while self.now_s() < t_end and not (stop_file and os.path.exists(stop_file)):
            if twist is not None:
                self.kb_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            self.process()

    def rotate(self, ang, wz, acc):
        """原地转 ang（rad，带符号）。按减速距离 wz²/(2·acc) 提前收油门。"""
        brake = wz * wz / (2 * acc)
        tw = Twist()
        tw.angular.z = math.copysign(wz, ang)
        acc0 = self.turn_acc
        t0 = self.now_s()
        while abs(self.turn_acc - acc0) < abs(ang) - brake and self.now_s() - t0 < abs(ang) / wz + 5:
            self.kb_pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.process()
        return self.now_s()


def errs(s):
    """一个样本的定位误差：(位置 m, 角度 rad 带符号)。"""
    return math.hypot(s[5] - s[1], s[6] - s[2]), wrap(s[7] - s[3])


def trial_metrics(seg, t_cmd_stop, tol_pos, tol_yaw):
    # 停稳 = 收油门后真值角速度第一次 < 0.05 rad/s
    t_still = seg[-1][0]
    for a, b in zip(seg, seg[1:]):
        if a[0] >= t_cmd_stop and b[0] > a[0] and abs(wrap(b[3] - a[3])) / (b[0] - a[0]) < 0.05:
            t_still = b[0]
            break
    moving = [s for s in seg if s[0] < t_still]
    after = [s for s in seg if s[0] >= t_still]
    # 里程计：相对开转那一帧，转了多少和真值差多少
    s0 = seg[0]
    odom_dev = [wrap((s[4] - s0[4]) - (s[3] - s0[3])) for s in seg]
    turn = sum(wrap(b[3] - a[3]) for a, b in zip(seg, seg[1:]))
    # 恢复：从停稳起，第一个"此后一直在容差内"的时刻
    rec = None
    for i in range(len(after)):
        if all(errs(s)[0] < tol_pos and abs(errs(s)[1]) < tol_yaw for s in after[i:]):
            rec = after[i][0] - t_still
            break
    # slam 修正次数：map→odom 角度跳变 > 0.05°
    mo = [wrap(s[7] - s[4]) for s in seg]
    jumps = sum(1 for a, b in zip(mo, mo[1:]) if abs(wrap(b - a)) > math.radians(0.05))
    d = math.degrees
    e_still = errs(after[0]) if after else (float("nan"), float("nan"))
    e_end = errs(seg[-1])
    return dict(
        turn_deg=round(d(turn), 1),
        loc_yaw_peak=round(d(max(abs(errs(s)[1]) for s in moving)), 2) if moving else None,
        loc_pos_peak=round(max(errs(s)[0] for s in moving) * 100, 1) if moving else None,
        loc_yaw_still=round(d(e_still[1]), 2), loc_pos_still=round(e_still[0] * 100, 1),
        loc_yaw_end=round(d(e_end[1]), 2), loc_pos_end=round(e_end[0] * 100, 1),
        recover_s=None if rec is None else round(rec, 2),
        odom_yaw_peak=round(d(max(abs(v) for v in odom_dev)), 2),
        odom_yaw_end=round(d(odom_dev[-1]), 2),
        slam_updates=jumps, n=len(seg))


def series_stats(ss, tol_pos, tol_yaw):
    if not ss:
        return None
    pe = [errs(s)[0] * 100 for s in ss]
    ye = [abs(math.degrees(errs(s)[1])) for s in ss]
    ok = sum(1 for p, y in zip(pe, ye) if p < tol_pos * 100 and y < math.degrees(tol_yaw))
    r = lambda v: None if v is None else round(v, 2)
    return dict(n=len(ss), yaw_p50=r(pct(ye, 0.5)), yaw_p95=r(pct(ye, 0.95)), yaw_max=r(max(ye)),
                pos_p50=r(pct(pe, 0.5)), pos_p95=r(pct(pe, 0.95)), pos_max=r(max(pe)),
                in_tol=round(ok / len(ss), 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("mode", choices=["spin", "watch"])
    ap.add_argument("--angles", default="[90,180,360]", help="spin：每次转的角度（度）")
    ap.add_argument("--reps", type=int, default=3, help="spin：每个角度几次（方向交替）")
    ap.add_argument("--wz", type=float, default=1.6, help="spin：角速度 rad/s（RPP 原地转同值）")
    ap.add_argument("--acc", type=float, default=2.4, help="spin：底盘角加速度上限，用来算提前收油门")
    ap.add_argument("--settle", type=float, default=3.0, help="spin：收油门后观察几秒")
    ap.add_argument("--duration", type=float, default=60.0, help="watch：旁听几秒（仿真时间）")
    ap.add_argument("--stop-file", help="watch：这个文件一出现就提前收尾（配合跑圈脚本结束）")
    ap.add_argument("--tol-deg", type=float, default=2.0)
    ap.add_argument("--tol-cm", type=float, default=3.0)
    a = ap.parse_args()
    tol_pos, tol_yaw = a.tol_cm / 100, math.radians(a.tol_deg)

    rclpy.init()
    n = LocErr()
    while n.now_s() == 0:   # 等 /clock，否则仿真时间从 0 跳过来，spin_for 直接结束
        rclpy.spin_once(n, timeout_sec=0.05)
    n.spin_for(2.0)
    if not n.samples:
        print("没收到真值或 TF：检查 /gz/model/my_bot/pose、map→base_footprint")
        return
    pre = n.samples[-30:]
    out = dict(label=a.label, mode=a.mode, pre_still=series_stats(pre, tol_pos, tol_yaw))
    print(f"开测前静止误差：{out['pre_still']}")

    if a.mode == "spin":
        trials = []
        zero = Twist()
        for ang in json.loads(a.angles):
            for k in range(a.reps):
                sign = 1 if k % 2 == 0 else -1
                i0 = len(n.samples)
                t_stop = n.rotate(sign * math.radians(ang), a.wz, a.acc)
                n.spin_for(a.settle, zero)
                m = trial_metrics(n.samples[i0:], t_stop, tol_pos, tol_yaw)
                m.update(cmd_deg=sign * ang)
                trials.append(m)
                print(f"{sign * ang:+5d}° 实转{m['turn_deg']:+7.1f}° | 定位 转动中最大 {m['loc_yaw_peak']}° "
                      f"{m['loc_pos_peak']}cm  停稳 {m['loc_yaw_still']}° {m['loc_pos_still']}cm  "
                      f"结束 {m['loc_yaw_end']}° {m['loc_pos_end']}cm  恢复 {m['recover_s']}s | "
                      f"里程计 最大 {m['odom_yaw_peak']}° 结束 {m['odom_yaw_end']}° | slam 修正 {m['slam_updates']} 次")
        out["trials"] = trials
        for ang in json.loads(a.angles):
            ts = [t for t in trials if abs(t["cmd_deg"]) == ang]
            rec = [t["recover_s"] for t in ts]
            print(f"[{ang}°×{len(ts)}] 转动中最大角度误差 中位 {pct([t['loc_yaw_peak'] or 0 for t in ts], 0.5)}° | "
                  f"停稳误差 最大 {max(abs(t['loc_yaw_still']) for t in ts)}° | 恢复 {rec} | "
                  f"里程计最大 {max(t['odom_yaw_peak'] for t in ts)}°")
    else:
        i0 = len(n.samples)
        n.spin_for(a.duration, stop_file=a.stop_file)
        ss = n.samples[i0:]
        rate = [0.0] + [abs(wrap(b[3] - a_[3])) / max(b[0] - a_[0], 1e-3) for a_, b in zip(ss, ss[1:])]
        spd = [0.0] + [math.hypot(b[1] - a_[1], b[2] - a_[2]) / max(b[0] - a_[0], 1e-3) for a_, b in zip(ss, ss[1:])]
        out["all"] = series_stats(ss, tol_pos, tol_yaw)
        out["turning"] = series_stats([s for s, w in zip(ss, rate) if w > 1.0], tol_pos, tol_yaw)
        out["still"] = series_stats([s for s, w, v in zip(ss, rate, spd) if w < 0.05 and v < 0.01], tol_pos, tol_yaw)
        for k in ("all", "turning", "still"):
            print(f"{k:8s} {out[k]}")
    out["series"] = [[round(v, 4) for v in s] for s in n.samples]
    json.dump(out, open(f"{a.label}_loc.json", "w"))
    print(f"写入 {a.label}_loc.json")
    n.kb_pub.publish(Twist())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
