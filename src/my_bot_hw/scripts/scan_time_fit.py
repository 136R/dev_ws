#!/usr/bin/env python3
# scan_time_fit.py — 标定 /scan 每束的真实采样时刻：t_i = stamp + c + s·k_i·time_increment
#   s=+1: k_i = i（序号递增 = 时刻递增）   s=-1: k_i = M-1-i（倒序）   s=0: 整帧同一时刻（只挪时间戳、不去畸变）
#
# 原理：车静止时的帧没有畸变、和时间模型无关，拿它当参照。运动中的帧按假设时刻查 odom→laser 位姿投到 odom 系，
# 模型对了就和前后最近的静止帧墙面重合（点到墙面距离最小）；网格搜索 (s, c)。odom 由 EKF 发（偏航来自 IMU），
# 几秒内足够准，拿它当参照。采集时要"停—转—停—直行—停"交替，让每段运动前后都有静止帧。
# 用法:
#   scan_time_fit.py fit <bag目录> [--scan-topic /scan]   # bag 要有 scan、/tf、/tf_static；车要有原地转和直行
#   scan_time_fit.py selftest                              # 合成数据：已知 (s, c) 能否找回
#
# 依赖 scipy（cKDTree）。开发板 numpy/scipy 版本不配套，bag 拷到开发机上跑。
import argparse
import math
import sys

import numpy as np
from scipy.spatial import cKDTree

MAX_RANGE = 4.0        # 只用 4 m 内的点：远处点稀、角度误差放大
CLIP = 0.10            # 单点最近距离截断（m），防止新看到的区域把误差拉爆
C_COARSE = np.arange(-0.25, 0.2501, 0.01)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Data:
    """scans: [(stamp, angle_min, angle_inc, time_inc, range_min, range_max, ranges)]
    odom: (t, x, y, yaw 已展开) 的 odom→base_footprint；laser: base_footprint→laser 的 (x, y, yaw)"""

    def __init__(self, scans, odom, laser):
        self.scans, self.laser = scans, laser
        self.ot, self.ox, self.oy, self.oyaw = odom

    def pose(self, t):
        return (np.interp(t, self.ot, self.ox), np.interp(t, self.ot, self.oy),
                np.interp(t, self.ot, self.oyaw))

    def speed(self, t, h=0.05):
        x0, y0, a0 = self.pose(t - h)
        x1, y1, a1 = self.pose(t + h)
        return math.hypot(x1 - x0, y1 - y0) / (2 * h), (a1 - a0) / (2 * h)

    def world_points(self, k, s, c):
        stamp, amin, ainc, tinc, rmin, rmax, r = self.scans[k]
        m = len(r)
        idx = np.arange(m)
        ok = np.isfinite(r) & (r > rmin) & (r < min(rmax, MAX_RANGE))
        idx = idx[ok]
        kk = idx if s >= 0 else (m - 1 - idx)
        t = stamp + c + (s != 0) * kk * tinc
        x, y, yaw = self.pose(t)
        lx, ly, lyaw = self.laser
        a = amin + idx * ainc
        px, py = r[ok] * np.cos(a), r[ok] * np.sin(a)
        # 先激光系 → base 系，再 base 系 → odom 系
        bx = lx + math.cos(lyaw) * px - math.sin(lyaw) * py
        by = ly + math.sin(lyaw) * px + math.cos(lyaw) * py
        return np.column_stack((x + np.cos(yaw) * bx - np.sin(yaw) * by,
                                y + np.sin(yaw) * bx + np.cos(yaw) * by))


def pick_pairs(d, per_class=80, max_gap=4.0):
    """运动帧 k 配前后最近的静止帧 j（|t_k - t_j| < max_gap）：rot 原地转 / lin 直行；
    still = 两个相隔 ≥1 s 的静止帧，量噪声本底"""
    state = []
    for k in range(len(d.scans)):
        t = d.scans[k][0] + 0.05
        if t < d.ot[0] + 0.3 or t > d.ot[-1] - 0.3:
            state.append(None)
            continue
        v, w = d.speed(t)
        state.append('rot' if abs(w) > 0.3 and v < 0.05 else 'lin' if v > 0.15 and abs(w) < 0.3
                     else 'still' if v < 0.005 and abs(w) < 0.02 else None)
    still = [k for k, st in enumerate(state) if st == 'still']
    ts = np.array([d.scans[k][0] for k in still])
    cls = {'rot': [], 'lin': [], 'still': []}
    for k, st in enumerate(state):
        if st in ('rot', 'lin') and len(still):
            j = int(np.argmin(np.abs(ts - d.scans[k][0])))
            if abs(ts[j] - d.scans[k][0]) < max_gap:
                cls[st].append((k, still[j]))
    for i, k in enumerate(still):
        later = [j for j in still[i:] if d.scans[j][0] - d.scans[k][0] >= 1.0]
        if later:
            cls['still'].append((later[0], k))
    for key, ks in cls.items():
        if len(ks) > per_class:
            cls[key] = [ks[i] for i in np.linspace(0, len(ks) - 1, per_class).astype(int)]
    return cls


_ref_cache = {}


def reference(d, j):
    """静止帧 j 的点云 + 每点墙面法向（邻域 PCA）；静止帧与模型无关，缓存"""
    if j not in _ref_cache:
        pts = d.world_points(j, 0, 0.0)
        tree = cKDTree(pts)
        nrm = np.zeros_like(pts)
        good = np.zeros(len(pts), bool)
        for i, nb in enumerate(tree.query_ball_point(pts, 0.06)):
            if len(nb) >= 4:
                q = pts[nb] - pts[nb].mean(0)
                ev, evec = np.linalg.eigh(q.T @ q)
                if ev[0] < 0.1 * ev[1]:          # 邻域够"直"才算墙面
                    nrm[i], good[i] = evec[:, 0], True
        _ref_cache[j] = (pts, tree, nrm, good)
    return _ref_cache[j]


def pair_error(d, pairs, s, c):
    """各帧对：运动帧每点到参照帧最近墙面的距离（截断后取中位数，cm），再对帧对求平均"""
    if not pairs:
        return float('nan')
    errs = []
    for k, j in pairs:
        pts, tree, nrm, good = reference(d, j)
        b = d.world_points(k, s, c)
        if len(pts) < 50 or len(b) < 50:
            continue
        dist, idx = tree.query(b)
        line = np.abs(np.einsum('ij,ij->i', b - pts[idx], nrm[idx]))
        e = np.where(good[idx], line, dist)
        errs.append(np.median(np.minimum(e, CLIP)))
    return 100 * float(np.mean(errs)) if errs else float('nan')


def fit(d):
    cls = pick_pairs(d)
    moving = cls['rot'] + cls['lin']
    print(f"帧对: 原地转 {len(cls['rot'])}  直行 {len(cls['lin'])}  静止 {len(cls['still'])}"
          f"  （共 {len(d.scans)} 帧，time_increment {d.scans[0][3]*1e6:.1f} µs，每帧 {len(d.scans[0][6])} 束）")
    if len(moving) < 10:
        sys.exit('运动帧对太少：要有原地转和直行')
    best = {}
    for s in (+1, -1, 0):
        coarse = [(pair_error(d, moving, s, c), c) for c in C_COARSE]
        e0, c0 = min(coarse)
        fine = [(pair_error(d, moving, s, c), c) for c in np.arange(c0 - 0.02, c0 + 0.0201, 0.002)]
        best[s] = min(fine)[::-1]   # (c, err)
    name = {+1: '正序', -1: '倒序', 0: '整帧同一时刻'}
    raw = {k: pair_error(d, cls[k], 0, 0.0) for k in cls}
    print(f"\n原始（下游现在的假设：整帧都在 stamp）: 原地转 {raw['rot']:.2f} cm  直行 {raw['lin']:.2f} cm"
          f"  静止本底 {raw['still']:.2f} cm")
    print('\n模型          最优 c(ms)   运动帧对误差  | 原地转   直行')
    for s in (+1, -1, 0):
        c, e = best[s]
        print(f"{name[s]:<10} {c*1000:+8.1f}      {e:6.2f} cm   | {pair_error(d, cls['rot'], s, c):5.2f}  "
              f"{pair_error(d, cls['lin'], s, c):5.2f}")
    sb = min(best, key=lambda s: best[s][1])
    print(f"\n结论: {name[sb]}，c = {best[sb][0]*1000:+.1f} ms"
          f"（第一束 = 本帧最早测到的束，在 stamp {best[sb][0]*1000:+.1f} ms）")
    return sb, best[sb][0]


def load_bag(path, scan_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    scans, odom, static = [], [], {}
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic not in (scan_topic, '/tf', '/tf_static'):
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        if topic == scan_topic:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            scans.append((st, m.angle_min, m.angle_increment, m.time_increment,
                          m.range_min, m.range_max, np.array(m.ranges, dtype=float)))
            laser_frame = m.header.frame_id
        else:
            for tr in m.transforms:
                if topic == '/tf_static':
                    static[tr.child_frame_id] = (tr.header.frame_id, tr.transform)
                elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                    st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                    odom.append((st, tr.transform.translation.x, tr.transform.translation.y,
                                 yaw_of(tr.transform.rotation)))
    # 静态链 laser → … → base_footprint，按 2D 合成（雷达水平安装）
    x = y = yaw = 0.0
    f = laser_frame
    while f != 'base_footprint':
        parent, t = static[f]
        ty = yaw_of(t.rotation)
        x, y = (t.translation.x + math.cos(ty) * x - math.sin(ty) * y,
                t.translation.y + math.sin(ty) * x + math.cos(ty) * y)
        yaw, f = yaw + ty, parent
    odom.sort()
    o = np.array(odom)
    return Data(scans, (o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3])), (x, y, yaw))


def selftest():
    """合成：4×3 m 房间 + 立柱，车先原地往复转再直行；按已知 (s=-1, c=+60 ms) 逐束光线求交生成 scan"""
    rng = np.random.default_rng(0)
    walls = [((-2, -1.5), (2, -1.5)), ((2, -1.5), (2, 1.5)), ((2, 1.5), (-2, 1.5)), ((-2, 1.5), (-2, -1.5)),
             ((0.8, 0.4), (1.1, 0.4)), ((1.1, 0.4), (1.1, 0.7)), ((1.1, 0.7), (0.8, 0.7)), ((0.8, 0.7), (0.8, 0.4))]
    t = np.arange(0, 24, 0.005)
    ph = t % 6.0                                          # 每 6 s：停 1 / 转 1.6 / 停 1 / 直行 1.4 / 停 1
    sgn = np.where((t // 6.0) % 2 == 0, 1.0, -1.0)
    w = np.where((ph > 1) & (ph < 2.6), sgn * 1.0 * np.sin(np.pi * (ph - 1) / 1.6), 0.0)
    v = np.where((ph > 3.6) & (ph < 5.0), sgn * 0.4 * np.sin(np.pi * (ph - 3.6) / 1.4), 0.0)
    yaw = np.cumsum(w) * 0.005
    x = -0.5 + np.cumsum(v * np.cos(yaw)) * 0.005
    y = np.cumsum(v * np.sin(yaw)) * 0.005
    laser = (-0.047, 0.0, math.pi)
    m, period, s_true, c_true = 720, 0.1, -1, 0.06
    tinc = period / (m - 1)
    d = Data([], (t, x, y, yaw), laser)
    amin, ainc = -math.pi, 2 * math.pi / (m - 1)
    for stamp in np.arange(0.5, 23.0, period):
        idx = np.arange(m)
        ti = stamp + c_true + (m - 1 - idx if s_true < 0 else idx) * tinc
        px, py, pyaw = d.pose(ti)
        lxw = px + np.cos(pyaw) * laser[0] - np.sin(pyaw) * laser[1]
        lyw = py + np.sin(pyaw) * laser[0] + np.cos(pyaw) * laser[1]
        ang = pyaw + laser[2] + amin + idx * ainc
        dx, dy = np.cos(ang), np.sin(ang)
        r = np.full(m, np.inf)
        for (ax, ay), (bx, by) in walls:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                tt = ((ax - lxw) * ey - (ay - lyw) * ex) / den
                uu = ((ax - lxw) * dy - (ay - lyw) * dx) / den
            hit = (np.abs(den) > 1e-9) & (tt > 0) & (uu >= 0) & (uu <= 1)
            r = np.where(hit & (tt < r), tt, r)
        r = r + rng.normal(0, 0.005, m)
        d.scans.append((stamp, amin, ainc, tinc, 0.05, 12.0, r))
    s, c = fit(d)
    ok = s == s_true and abs(c - c_true) < 0.01
    print(f"\n自检: 真值 倒序 c=+60.0 ms → 找回 {'倒序' if s < 0 else '正序' if s > 0 else '整帧'} "
          f"c={c*1000:+.1f} ms  {'✅ 通过' if ok else '❌ 失败'}")
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('fit')
    f.add_argument('bag')
    f.add_argument('--scan-topic', default='/scan')
    sub.add_parser('selftest')
    a = p.parse_args()
    if a.cmd == 'selftest':
        sys.exit(selftest())
    fit(load_bag(a.bag, a.scan_topic))


if __name__ == '__main__':
    main()
