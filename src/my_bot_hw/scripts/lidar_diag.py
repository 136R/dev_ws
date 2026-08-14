#!/usr/bin/env python3
# lidar_diag.py — 2D 激光链路诊断：立柱核对 / 噪声本底 / 时延与帧内畸变
#
# 配套 spec: docs/spec/2026-08-14-2D激光链路优化.md（阶段 A 的全部测量）
#
# 用法:
#   ./lidar_diag.py record --duration 60 --out ~/a0.npz               # A0/A1 静止采集
#   ./lidar_diag.py verify ~/a0.npz                                   # A0 立柱核对
#   ./lidar_diag.py noise  ~/a0.npz                                   # A1 噪声本底 + 滤波阈值
#   ./lidar_diag.py record --duration 45 --with-odom --out ~/a2.npz   # A2 摆动采集
#   ./lidar_diag.py tshift ~/a2.npz                                   # A2/C1 时间戳偏移标定 ← 用这个
#   ./lidar_diag.py delay  ~/a2.npz                                   # ⚠ 时间模型已被推翻，仅历史对照
#   ./lidar_diag.py compare ~/b_raw.npz ~/b_filt.npz --keep-sector=-20:20   # B 验收：滤波前后对比
#
# ⚠ 不要把结果存进 /tmp —— 开发板的 /tmp 是 tmpfs，重启即失（2026-08-14 丢过一次数据）。
#
# ⚠ 角度坐标系（最容易搞错的地方）：
#   lidar_yaw ≈ π，即 laser_frame 的 x 轴与 base_link 的 x 轴**反向**。
#   → laser_filters 的 AngularBounds 参数用的是 **laser_frame 角度**（它直接操作 scan 索引）；
#   → 人判断"是不是挡住了正前方"用的是 **base_link 角度**。
#   本脚本两个都打印，配置里只准填 laser_frame 那一列。
#
# ⚠ verify 是**核对**不是**检测**：立柱位置由 CAD 尺寸算出来，脚本只回答"这几个位置上
#   实测到了什么"。先前那版靠点云特征去盲找柱子，在真实环境里捞出 8 个低反射率物体当柱子、
#   真柱子反而被有效率判据筛掉了 —— 尺寸已知就不该盲找。

import argparse
import math
import sys
import warnings

import numpy as np

# 全遮挡的束整列都是 NaN，nanmedian/nanstd 会对每一列刷一条 RuntimeWarning。
# 这些列正是我们要看的东西，不是异常 —— 静音掉，否则真正的告警会被淹没。
warnings.filterwarnings('ignore', category=RuntimeWarning, module='numpy')

# 与 robot_bringup.launch.py:47 保持一致；改了外参记得同步或用 --lidar-yaw 覆盖
DEFAULT_LIDAR_YAW = 3.115414361

# 立柱几何（CAD，2026-08-14 用户提供 + 更正）。改了实物记得同步这三行。
PILLAR_R_CENTER = 0.10253      # 柱心到底盘圆心(=雷达中心)的距离 (m)
PILLAR_RADIUS = 0.003          # 柱半径 (m)，⌀6 mm
PILLAR_BEARINGS = '45,135,-135,-45'   # base_link 下的方位 (deg)


# ─────────────────────────── 采集 ───────────────────────────

def cmd_record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry

    sensor_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=50,
    )

    class Recorder(Node):
        def __init__(self):
            # 节点名带上话题：B 阶段验收要**同时**采 /scan 和 /scan_filtered
            # （先后采会把场景漂移算成滤波效果，2026-08-14 踩过），两个进程不能重名。
            suffix = ''.join(ch if ch.isalnum() else '_' for ch in args.scan_topic).strip('_')
            super().__init__(f'lidar_diag_recorder_{suffix}')
            self.scans, self.scan_t = [], []
            self.meta = None
            self.odom_t, self.odom_yaw = [], []
            self.imu_t, self.imu_gz = [], []
            self.create_subscription(LaserScan, args.scan_topic, self._scan_cb, sensor_qos)
            if args.with_odom:
                self.create_subscription(Odometry, args.odom_topic, self._odom_cb, sensor_qos)
                self.create_subscription(Imu, args.imu_topic, self._imu_cb, sensor_qos)

        def _scan_cb(self, m):
            if self.meta is None:
                self.meta = dict(
                    angle_min=m.angle_min, angle_max=m.angle_max,
                    angle_increment=m.angle_increment,
                    scan_time=m.scan_time, time_increment=m.time_increment,
                    range_min=m.range_min, range_max=m.range_max,
                )
            self.scans.append(np.asarray(m.ranges, dtype=np.float32))
            self.scan_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

        def _odom_cb(self, m):
            q = m.pose.pose.orientation
            self.odom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.odom_yaw.append(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))

        def _imu_cb(self, m):
            self.imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.imu_gz.append(m.angular_velocity.z)

    rclpy.init()
    node = Recorder()
    t_end = node.get_clock().now().nanoseconds * 1e-9 + args.duration
    print(f'采集中… {args.duration}s，话题 {args.scan_topic}', file=sys.stderr)
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    n = len(node.scans)
    node.destroy_node()
    rclpy.shutdown()

    if n == 0:
        sys.exit(f'没收到任何 {args.scan_topic} —— 雷达没起，或 QoS/DOMAIN_ID 不对')
    # 变长帧（angle_compensate 关闭时会出现）截齐到最短，避免 ragged array
    m_len = min(len(s) for s in node.scans)
    ranges = np.stack([s[:m_len] for s in node.scans])
    np.savez_compressed(
        args.out, ranges=ranges, scan_t=np.asarray(node.scan_t),
        odom_t=np.asarray(node.odom_t), odom_yaw=np.asarray(node.odom_yaw),
        imu_t=np.asarray(node.imu_t), imu_gz=np.asarray(node.imu_gz),
        **node.meta)
    print(f'已存 {args.out}：{n} 帧 × {m_len} 束'
          f'{f"，odom {len(node.odom_t)} 条" if args.with_odom else ""}')


# ─────────────────────────── 公共工具 ───────────────────────────

class Capture:
    """把 npz 里的东西整理成分析用的形状。"""

    def __init__(self, path, lidar_yaw):
        d = np.load(path)
        self.r = d['ranges'].astype(np.float64)              # (N, M)
        self.t = d['scan_t']
        self.n, self.m = self.r.shape
        self.angle_min = float(d['angle_min'])
        self.dth = float(d['angle_increment'])
        self.scan_time = float(d['scan_time'])
        self.time_increment = float(d['time_increment'])
        self.range_min = float(d['range_min'])
        self.range_max = float(d['range_max'])
        self.lidar_yaw = lidar_yaw
        for k in ('odom_t', 'odom_yaw', 'imu_t', 'imu_gz'):
            setattr(self, k, d[k] if k in d else np.array([]))
        # laser_frame 下每束的角度
        self.ang = self.angle_min + np.arange(self.m) * self.dth
        # 有效性：inf / nan / 0 / 低于物理盲区 一律不算命中
        self.valid = np.isfinite(self.r) & (self.r > max(self.range_min, 1e-3))

    def to_base(self, a):
        """laser_frame 角度 → base_link 角度（绕 z 转 lidar_yaw），归一到 (-π, π]。"""
        return wrap(a + self.lidar_yaw)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def fit_wall_normal(x, y):
    """对一组点做 total least squares 直线拟合，返回法向角(rad)与 RMS 残差(m)。"""
    if len(x) < 8:
        return None, None
    pts = np.stack([x - x.mean(), y - y.mean()])
    # 协方差最小特征向量 = 法向
    w, v = np.linalg.eigh(pts @ pts.T)
    nx, ny = v[:, 0]
    resid = float(np.sqrt(np.mean((pts.T @ v[:, 0]) ** 2)))
    return math.atan2(ny, nx), resid


# ─────────────────────────── A0 立柱核对 ───────────────────────────

def cmd_verify(args):
    """按 CAD 已知的立柱位置逐个核对。不做检测 —— 位置是算出来的，不是找出来的。"""
    c = Capture(args.npz, args.lidar_yaw)
    R, rp = args.pillar_r, args.pillar_radius
    half = math.asin(min(rp / R, 1.0))
    tol = math.radians(args.tol)
    bearings = [float(x) for x in args.expect.split(',')]

    rate = c.valid.mean(axis=0)
    with np.errstate(all='ignore'):
        med = np.nanmedian(np.where(c.valid, c.r, np.nan), axis=0)

    d_near, d_far = R - rp, math.sqrt(max(R * R - rp * rp, 0.0))

    print(f'== A0 立柱核对 ==  {c.n} 帧 × {c.m} 束，角分辨率 {math.degrees(c.dth):.3f}°/束')
    print(f'   CAD: R柱={R*1000:.2f} mm, r柱={rp*1000:.1f} mm '
          f'⇒ 遮挡半角 {math.degrees(half):.3f}°、全角 {math.degrees(2*half):.3f}°、'
          f'四柱合计 {math.degrees(8*half):.2f}°')
    print(f'   CAD: 柱面回波应落在 {d_near:.4f} ~ {d_far:.4f} m')
    print(f'   窗口 = 预期中心 ± ({math.degrees(half):.2f}° + {args.tol:.1f}° 余量)，'
          f'约 {2*int((half+tol)/c.dth)+1} 束/柱\n')

    # 「柱子回波」判据：落在 CAD 预期距离区间内（放 ±20 mm，覆盖装配误差与测距噪声）
    okmask = c.valid & (c.r > d_near - 0.02) & (c.r < d_far + 0.02)
    n_expect = 2.0 * half / c.dth          # CAD 预期的柱子束数
    in_win = np.zeros(c.m, dtype=bool)
    hit = 0
    print(f'{"#":>2} {"base_link":>10} {"laser_frame":>12} {"窗口束":>6} '
          f'{"柱束/预期":>10} {"柱束命中率":>11} {"回波min":>8} {"回波中位":>9} '
          f'{"回波max":>8} {">阈值":>7}')
    for k, b in enumerate(bearings, 1):
        ctr = wrap(math.radians(b) - c.lidar_yaw)
        sel = np.abs(wrap(c.ang - ctr)) < (half + tol)
        in_win |= sel
        idx = np.flatnonzero(sel)
        # 逐束统计"测到柱子距离的帧比例"。柱子回波是间歇的（实测约 17%），
        # 所以不能看它占窗口全部回波的比例——窗口里还有背景墙，会把它稀释掉。
        per_beam = okmask[:, idx].mean(axis=0)
        pb = per_beam > args.beam_hit
        vals = c.r[:, idx][okmask[:, idx]]
        ok = pb.sum() >= max(1.0, args.confirm * n_expect)
        hit += ok
        if vals.size == 0:
            print(f'{k:>2} {b:>+9.1f}° {math.degrees(ctr):>+11.2f}° {len(idx):>6} '
                  f'{0:>4}/{n_expect:>4.1f} {"—":>11} {"—":>8} {"无柱子回波":>9} '
                  f'{"—":>8} {"—":>7}')
            continue
        over = float((vals > args.lower_threshold).mean())
        print(f'{k:>2} {b:>+9.1f}° {math.degrees(ctr):>+11.2f}° {len(idx):>6} '
              f'{pb.sum():>4}/{n_expect:>4.1f} {per_beam[pb].mean()*100:>10.1f}% '
              f'{np.min(vals):>8.4f} {np.median(vals):>9.4f} {np.max(vals):>8.4f} '
              f'{over*100:>6.1f}%')

    print(f'\n   「柱束」  = 窗口内有 >{args.beam_hit*100:.0f}% 的帧测到 '
          f'{d_near-0.02:.3f}~{d_far+0.02:.3f} m 的束数；预期 {n_expect:.1f} 束')
    print(f'   「命中率」= 这些柱束平均有多少比例的帧真的测到柱子'
          f'（远低于 100% 属正常：柱面在 C1 探测能力边缘）')
    print(f'   「>阈值」 = 柱子回波中超过 lower_threshold={args.lower_threshold:.3f} 的比例'
          f' ← spec F10 的关键数')

    # ── 判定 1：CAD 方位对不对 ──
    print(f'\n── 判定 1：CAD 方位与车头对应 ──')
    if hit == len(bearings):
        print(f'   ✓ 四个预期方位全部核对到柱子 —— CAD 的 dX/dZ 轴与车头方向对应正确，'
              f'H4 确认，不触发「雷达绕 z 转 45° 重装」')
    elif hit == 0:
        print(f'   ✗ 四个预期方位一个都没核对到 —— CAD 轴向与车头**不对应**，'
              f'或立柱不在扫描平面内。\n'
              f'     先用 --expect 0,90,180,-90 重跑一次；仍为 0 则 spec 的 F9 需要重查。')
    else:
        print(f'   ⚠ 只核对到 {hit}/{len(bearings)} 根 —— 装配不对称，或部分柱子被别的东西挡了。'
              f'看上表哪几行的「符合CAD」偏低。')

    # ── 判定 2：F10，回波是否骑在 lower_threshold 上 ──
    print(f'── 判定 2：F10 —— 柱子回波 vs lower_threshold={args.lower_threshold:.3f} ──')
    okd = c.r[:, in_win][okmask[:, in_win]]
    if okd.size == 0:
        print('   窗口内没有符合 CAD 的回波 —— F10 不适用（柱子回波全在盲区之下）')
    else:
        over = float((okd > args.lower_threshold).mean())
        print(f'   四柱合计 {okd.size} 个符合回波，**{over*100:.1f}% 超过阈值**'
              f'（会穿过 range filter 进入 /scan_filtered）')
        if 0.02 < over < 0.98:
            print(f'   ⚠ **骑在阈值上** —— F10 成立。这些点会变成 costmap 里的贴身假障碍，'
                  f'且穿过多少随装配误差/温漂跳变。\n'
                  f'     → 执行 B3a′：lower_threshold {args.lower_threshold:.2f} → '
                  f'{args.lower_threshold-0.01:.2f}，让回波稳定进入；\n'
                  f'     → 再由 B3a 的 polygon filter 按车体多边形精确删除。')
        elif over >= 0.98:
            print(f'   ⚠ 几乎全部穿过 —— 柱子回波正在直接污染 costmap，B3a 必做，B3a′ 不必')
        else:
            print(f'   ✓ 几乎全被砍掉 —— F10 的危害在本机不成立，B3a′ 不做，B3a 降级')

    # ── 反证：窗口之外还有没有近点 ──
    print(f'── 反证：四个窗口之外的近点（<{args.near_thresh:.2f} m）──')
    # 同样按"有多少比例的帧测到近点"判，不能用中位数 —— 间歇性近点会被背景值淹没
    near_rate = (c.valid & (c.r < args.near_thresh)).mean(axis=0)
    other = np.flatnonzero(~in_win & (near_rate > args.beam_hit))
    if other.size == 0:
        print('   ✓ 没有 —— 车身周围除立柱外无其他遮挡物，CAD 描述完整')
    else:
        print(f'   ⚠ {other.size} 束，说明还有 CAD 没描述的近距遮挡物：')
        for i in other[:12]:
            v = c.r[c.valid[:, i] & (c.r[:, i] < args.near_thresh), i]
            print(f'     base {math.degrees(c.to_base(c.ang[i])):>+7.1f}°  '
                  f'命中率 {near_rate[i]*100:>5.1f}%  距离中位 {np.median(v):.4f} m')
        if other.size > 12:
            print(f'     …另有 {other.size-12} 束')


# ─────────────────────────── A1 噪声本底 / 滤波阈值 ───────────────────────────

def cmd_noise(args):
    c = Capture(args.npz, args.lidar_yaw)
    rate = c.valid.mean(axis=0)
    rr = np.where(c.valid, c.r, np.nan)
    with np.errstate(all='ignore'):
        med = np.nanmedian(rr, axis=0)
        std = np.nanstd(rr, axis=0)

    # 只统计"稳定命中的中距束"，避免边缘/盲区污染本底
    sel = (rate > 0.95) & (med > 0.3) & (med < 5.0)
    print(f'== A1 噪声本底 ==  {c.n} 帧 × {c.m} 束，参与统计 {sel.sum()} 束')
    if sel.sum() == 0:
        sys.exit('没有稳定命中的中距束 —— 采集场景太空旷或太近，换个有墙有边缘的地方重采')
    print(f'   σ 中位数 {np.median(std[sel]) * 1000:.1f} mm，'
          f'90 分位 {np.percentile(std[sel], 90) * 1000:.1f} mm')
    print(f'   → 噪声本底 σ = {np.median(std[sel]):.4f} m（回填 spec 验收栏）\n')

    # shadow 角：与 laser_filters/ScanShadowsFilter 完全同式
    #   angle = atan2(r2·sin(Δθ), r1 − r2·cos(Δθ))，落在 [min_angle, max_angle] 外的点被删
    r1, r2 = c.r[:, :-1], c.r[:, 1:]
    ok = c.valid[:, :-1] & c.valid[:, 1:]
    with np.errstate(all='ignore'):
        sh = np.degrees(np.abs(np.arctan2(r2 * math.sin(c.dth), r1 - r2 * math.cos(c.dth))))
    sh = sh[ok]
    sh = sh[np.isfinite(sh)]
    print(f'== shadow 角分布 ==（{len(sh)} 个相邻点对）')
    for q in (0.1, 0.5, 1, 2, 5, 50, 95, 99):
        print(f'   {q:>4}% 分位: {np.percentile(sh, q):6.2f}°')
    p1 = np.percentile(sh, 1)
    print(f'   → 建议 ScanShadowsFilter: min_angle={max(5, int(p1)):d}, max_angle=180-同值')
    print('     （min_angle 取 1% 分位附近：只砍最掠射的那一小撮，别把墙角一起砍了）\n')

    # 相邻点距离跳变 → speckle 的 max_range_difference
    with np.errstate(all='ignore'):
        jump = np.abs(r1 - r2)[ok]
    jump = jump[np.isfinite(jump)]
    print(f'== 相邻点距离跳变 ==（决定 speckle 的 max_range_difference）')
    for q in (50, 90, 99, 99.5, 99.9):
        print(f'   {q:>5}% 分位: {np.percentile(jump, q):6.3f} m')
    print(f'   → 建议 max_range_difference ≈ {np.percentile(jump, 99.5):.2f} m'
          f'（现值 0.6，文档记 0.1 —— 见 spec F2）')
    print(f'   ⚠ 该建议随角分辨率变：本采集是 {math.degrees(c.dth):.2f}°/束，'
          f'分辨率越粗、相邻点天然跳变越大，别把粗分辨率的分位数用到细分辨率上')


# ─────────────────────────── A2/A3 时延与帧内畸变 ───────────────────────────

def cmd_delay(args):
    """⚠ 本子命令的 τ / t_eff / 分支判据在本机**不可信**，只保留给历史数据对照。

    它假设第 i 束的采样时刻 = header.stamp + i × time_increment。2026-08-14 实测证明
    **方向是反的**（sllidar 的 reverse_data 分支倒序写 ranges 却没处理时间），
    于是 t_eff 算错、τ 出现物理上不可能的负值、`τ + t_eff` 这个和也失去意义。

    要量「下游按 header.stamp 查 TF 的实际误差」，用 `~/lidar_diag/tcheck2.py`
    那套方法：沿整圈取多个索引窗，各自求「几何对应 odom yaw 在 stamp+Δ 的值」的 Δ，
    再对索引线性拟合。它不依赖任何符号约定，且能顺带验出时间方向对不对。
    """
    print('⚠ delay 的分支判据基于已被推翻的正向时间模型，结果仅供历史对照 —— '
          '见本函数 docstring 与 spec 的 A2 追加自证\n')
    c = Capture(args.npz, args.lidar_yaw)
    if len(c.odom_t) == 0:
        sys.exit('这份采集没有 odom —— A2 要用 `record --with-odom` 重采')

    # base_link 前方 ±sector° 对应的束（注意 lidar_yaw≈π 的翻转）
    a_base = c.to_base(c.ang)
    sec = np.abs(a_base) < math.radians(args.sector)
    print(f'== A2/A3 时延与帧内畸变 ==  {c.n} 帧，前方扇区 {sec.sum()} 束')

    odom_yaw_u = np.unwrap(c.odom_yaw)
    th, intra, tt, t_eff = [], [], [], []
    for i in range(c.n):
        v = c.valid[i] & sec
        if v.sum() < 16:
            continue
        idx = np.flatnonzero(v)
        x = c.r[i, idx] * np.cos(c.ang[idx])
        y = c.r[i, idx] * np.sin(c.ang[idx])
        n_all, resid = fit_wall_normal(x, y)
        if n_all is None or resid > args.max_resid:
            continue
        # A3：按束在帧内的**实际采样时刻**分早/晚两组各拟一次。
        # 关键：lidar_yaw≈π 使前方扇区正好落在帧的**首尾两端**（索引 0 附近 + 索引 M 附近），
        # 时间上分别是帧最早和帧最晚的一批。按"点数对半分"会把帧首的点混进晚组 ——
        # 必须按 tj 的**最大间隙**切开，否则 dt 与 Δθ 都会系统性偏小。
        tj = idx * c.time_increment
        gaps = np.diff(tj)
        k = int(np.argmax(gaps))
        if gaps.size and gaps[k] > 3 * np.median(gaps):
            cut = k + 1                      # 扇区跨帧首尾：在断裂处切
        else:
            cut = len(idx) // 2              # 扇区在帧内连续：对半切
        n_a, _ = fit_wall_normal(x[:cut], y[:cut])
        n_b, _ = fit_wall_normal(x[cut:], y[cut:])
        dt = tj[cut:].mean() - tj[:cut].mean()
        if n_a is None or n_b is None or dt < 1e-6:
            continue
        th.append(n_all)
        t_eff.append(tj.mean())              # 整帧拟合真正代表的时刻（相对 header.stamp）
        intra.append(wrap(n_b - n_a) / dt * c.scan_time)   # 折算成整帧的畸变角
        tt.append(c.t[i])

    if len(th) < 20:
        sys.exit(f'只拟出 {len(th)} 帧墙面 —— 车没正对墙，或 --max-resid 太严')
    th = np.asarray(th)
    tt = np.asarray(tt)
    intra = np.asarray(intra)
    t_eff = np.asarray(t_eff)

    # 法向有 π 周期：先倍角解缠再折半
    th = np.unwrap(th * 2) / 2
    # 守卫：帧间法向跳变过大 = 前方扇区换了一面墙，解缠已失效，整段数据作废
    jump = np.abs(np.diff(th))
    if jump.size and jump.max() > math.radians(args.max_jump):
        k = int(np.argmax(jump))
        sys.exit(
            f'第 {k}→{k+1} 帧墙面法向跳了 {math.degrees(jump.max()):.1f}°'
            f'（> --max-jump {args.max_jump}°）—— 前方扇区换了一面墙，法向解缠失效。\n'
            f'A2 必须**小幅往复摆动**（±15° 左右）而不是整圈旋转，'
            f'才能让前方扇区始终盯住同一面墙。')

    # 整帧拟合代表的不是 header.stamp，而是所用那批束的平均采样时刻。
    # 先把这个已知偏移加回去，回归斜率才是**纯传输延迟**，不掺"起始 vs 中心"的账。
    # 但若上游已做过 deskew（C2），所有点已重投影到同一时刻，这个偏移就不存在了 ——
    # 那时必须 --deskewed，否则会把 t_eff 整个误算进 τ。
    te = 0.0 if args.deskewed else float(np.median(t_eff))
    yaw_at_scan = np.interp(tt + te, c.odom_t, odom_yaw_u)
    omega = np.gradient(yaw_at_scan, tt)
    moving = np.abs(omega) > args.omega_min
    fwd, bwd = omega > args.omega_min, omega < -args.omega_min
    print(f'   正转 {fwd.sum()} 帧，反转 {bwd.sum()} 帧，'
          f'|ω| 中位 {np.median(np.abs(omega[moving])) if moving.any() else float("nan"):.3f} rad/s')
    if fwd.sum() < 10 or bwd.sum() < 10:
        sys.exit('正反转各需 ≥10 帧 —— 回归要靠 ω 取到正负两侧才能把常数偏置和 τ 分开')

    # 车转 yaw，墙面法向在 laser_frame 里反向转同样多，故 e ≡ −θ_wall − yaw 应为常数。
    # 若 scan 的时间戳滞后 τ，scan 反映的是 yaw(t−τ)：e(t) = a − τ·ω(t)。
    # 对 ω 线性回归：斜率 = −τ，截距 a 吸收掉任意参考偏移（故本函数不报静态外参残差，
    # 那要靠对墙摆正的绝对基准，见 spec F7 的 2026-08-13 标定）。
    e = -th - yaw_at_scan
    b, a = np.polyfit(omega, e, 1)
    tau = -b
    resid = e - (a + b * omega)
    ss = 1.0 - np.var(resid) / np.var(e) if np.var(e) > 0 else float('nan')
    print(f'\n   → **纯传输时延 τ = {tau * 1000:+.1f} ms**'
          f'  回归 R²={ss:.3f}，残差 RMS={math.degrees(np.std(resid)):.3f}°')
    print(f'   → 所用束的帧内平均时刻 t_eff = {te * 1000:.1f} ms'
          f'（相对 header.stamp；scan_time={c.scan_time * 1000:.1f} ms）')
    print(f'   → **下游按 header.stamp 查 TF 的实际误差 = τ + t_eff = '
          f'{(tau + te) * 1000:+.1f} ms** ← C1 要消掉的就是这个')
    if ss < 0.5:
        print('   ⚠ R² 偏低 —— τ 不可信。R² 低通常意味着 e 的变化几乎全是噪声，'
              '即 τ 本身接近 0；也可能是墙面拟合太脏或 odom 时间戳抖动。先看残差 RMS')

    intra_deg = math.degrees(np.median(np.abs(intra[moving]))) if moving.any() else float('nan')
    print(f'\n   → **帧内畸变角 Δθ_intra = {intra_deg:.2f}°**（中位，仅取运动帧，已折算成整帧）')
    print(f'     解析预期 |ω|·scan_time = '
          f'{math.degrees(np.median(np.abs(omega[moving])) * c.scan_time):.2f}°')

    print('\n── 分支判据（spec 阶段 A）──')
    tot = abs(tau + te)
    if tot > 0.3 * c.scan_time:
        print(f'   |τ+t_eff|={tot*1000:.1f}ms > 0.3×scan_time({0.3*c.scan_time*1000:.0f}ms)'
              f' —— **时延主导，先做 C1（挪时间戳），改完重跑本测**')
    elif tot < 0.010:
        # Δθ_intra 的噪声本底约 0.2~0.3°（墙面拟合抖动），低于 1° 视为畸变不显著
        if intra_deg < 1.0:
            print(f'   |τ+t_eff|={tot*1000:.1f}ms 且 Δθ_intra={intra_deg:.2f}° —— '
                  f'**两项都不显著，C 阶段无事可做**。墙重影另有来源，回去查 spec 假设 H2')
        else:
            print(f'   |τ+t_eff|={tot*1000:.1f}ms < 10ms —— **畸变主导，直接上 C2（deskew 节点）**')
    else:
        print(f'   |τ+t_eff|={tot*1000:.1f}ms 落在 10~30ms 模糊带 —— **回来重新出卡，不许拍脑袋选**')
    if intra_deg > 2.0:
        print(f'   Δθ_intra={intra_deg:.2f}° > 2° —— 帧内畸变本身显著，C1 只能消掉一阶项，'
              f'C2 迟早要做')
    elif not args.deskewed and moving.any():
        print(f'   ⚠ Δθ_intra={intra_deg:.2f}° 接近 0，但 --deskewed 未置位。'
              f'若这份数据来自已做 deskew 的链路，请加 --deskewed 重跑 —— '
              f'否则 t_eff({te*1000:.0f}ms) 会被整个误算进 τ')


# ──────────────── 时间戳偏移标定（取代 delay，A2/C1 的权威口径）────────────────

def cmd_tshift(args):
    """量「这一帧的几何，对应 odom yaw 在 header.stamp + Δ 的值」，Δ 随索引怎么变。

    为什么不用 delay：delay 假设第 i 束采样于 stamp + i×time_increment。实测方向是
    反的（sllidar 的 reverse_data 分支倒序写 ranges 却没处理时间），于是它的 τ / t_eff
    都算错。本子命令**不依赖任何符号约定**：沿整圈取多个索引窗，每个窗独立求最优 Δ，
    再对索引线性拟合 ——
      斜率 ÷ time_increment ≈ +1 ⇒ 驱动的帧内时间模型正确
      斜率 ÷ time_increment ≈ −1 ⇒ 数组相对时间倒序（本机就是这种）
      全帧均值 Δ = 下游按 header.stamp 查 TF 的实际误差 ← 要消掉的就是它

    采集要求同 A2：正对直墙 1.0~1.5 m，原地往复摆动，`record --with-odom`。
    """
    c = Capture(args.npz, args.lidar_yaw)
    if len(c.odom_t) == 0:
        sys.exit('这份采集没有 odom —— 要用 `record --with-odom` 重采')
    psi_u = np.unwrap(c.odom_yaw)
    grid = np.arange(-args.search, args.search + 1e-9, 0.001)
    print(f'== 时间戳偏移标定 ==  {c.n} 帧，scan_time={c.scan_time*1000:.1f} ms，'
          f'time_increment={c.time_increment*1e6:.1f} µs')
    print(f'   窗半宽 {args.half} 束（{(2*args.half+1)*math.degrees(c.dth):.1f}°）'
          f'，resid≤{args.max_resid} m，最少 {args.min_frames} 帧，'
          f'RMS≤{args.max_rms}°，跳变≤{args.max_jump}°')
    print(f'\n{"窗中心i":>7} {"base角":>8} {"拟出帧":>7} {"跳变max":>9} {"Δ(ms)":>9}  结果')

    rows, dropped = [], {}
    for k in range(args.half, c.m - args.half, args.step):
        idx0 = np.arange(k - args.half, k + args.half + 1)
        th, tt = [], []
        for i in range(c.n):
            idx = idx0[c.valid[i, idx0]]
            if len(idx) < max(8, args.half):
                continue
            x = c.r[i, idx] * np.cos(c.ang[idx])
            y = c.r[i, idx] * np.sin(c.ang[idx])
            n, resid = fit_wall_normal(x, y)
            if n is None or resid > args.max_resid:
                continue
            th.append(n)
            tt.append(c.t[i])
        base = math.degrees(float(c.to_base(np.array([c.ang[k]]))[0]))
        if len(th) < args.min_frames:
            print(f'{k:7d} {base:8.1f} {len(th):7d} {"-":>9} {"-":>9}  拟出帧不足')
            dropped['帧不足'] = dropped.get('帧不足', 0) + 1
            continue
        th = np.unwrap(np.asarray(th) * 2) / 2      # 法向有 π 周期
        tt = np.asarray(tt)
        jmp = math.degrees(np.abs(np.diff(th)).max())
        if jmp > args.max_jump:
            print(f'{k:7d} {base:8.1f} {len(th):7d} {jmp:8.1f}° {"-":>9}  法向跳变过大'
                  f'（窗里换了另一个物体）')
            dropped['跳变'] = dropped.get('跳变', 0) + 1
            continue
        var = np.array([np.var(-th - np.interp(tt + g, c.odom_t, psi_u)) for g in grid])
        j = int(np.argmin(var))
        rms = math.degrees(math.sqrt(var[j]))
        ok = rms <= args.max_rms
        print(f'{k:7d} {base:8.1f} {len(th):7d} {jmp:8.1f}° {grid[j]*1000:+9.1f}  '
              f'{"采用" if ok else f"RMS {rms:.2f}° 超限"}')
        if ok:
            rows.append((k, grid[j]))
        else:
            dropped['RMS'] = dropped.get('RMS', 0) + 1

    if len(rows) < 3:
        print(f'\n   采用窗不足 3 个。淘汰原因：{dropped}')
        print('   ⚠ 先看上面每行的 Δ 是不是已经排成一条直线 —— 若是，说明数据是好的，'
              '只是门限太严（近距/杂乱场景墙面拟合本就更噪），放宽 --max-rms 重跑。'
              '\n     2026-08-14 踩过：写死 RMS≤1.5° 把 20 个好窗全判成"没有直墙段"。')
        sys.exit(1)

    ks = np.array([r[0] for r in rows], float)
    ds = np.array([r[1] for r in rows])
    s, b = np.polyfit(ks, ds, 1)
    mid = b + s * (c.m - 1) / 2.0
    print(f'\n   采用 {len(rows)} 个窗'
          f'{"，淘汰 " + str(dropped) if dropped else ""}')
    print(f'   斜率 = {s*1e6:+.1f} µs/束（time_increment = {c.time_increment*1e6:.1f}）'
          f'  **比值 {s/c.time_increment:+.2f}**'
          f'  {"← 帧内时间倒序" if s < 0 else "← 帧内时间正序"}')
    print(f'   截距（i=0 处 Δ）= {b*1000:+.1f} ms，拟合残差 RMS = {np.std(ds-(b+s*ks))*1000:.1f} ms')
    print(f'\n   → **全帧均值 Δ = {mid*1000:+.1f} ms**'
          f'  ← 下游按 header.stamp 查 TF 的实际误差')
    if abs(mid) < 0.010:
        print(f'   ✓ |Δ| < 10 ms —— 时间戳偏移已校准')
    else:
        print(f'   ✗ |Δ| = {abs(mid)*1000:.1f} ms ≥ 10 ms —— '
              f'把 sllidar_node.cpp 的 kScanChainLatencyS 调整 {-mid*1000:+.1f} ms 后重编重测')
        print(f'     （见 docs/vendor/sllidar_ros2_本地修改.md）')


# ─────────────────────────── B 阶段验收：滤波前后对比 ───────────────────────────

def _shadow_deg(c):
    """相邻点对的 shadow 角（度）+ 有效掩码。与 ScanShadowsFilter 同式。"""
    r1, r2 = c.r[:, :-1], c.r[:, 1:]
    ok = c.valid[:, :-1] & c.valid[:, 1:]
    with np.errstate(all='ignore'):
        sh = np.degrees(np.abs(np.arctan2(r2 * math.sin(c.dth), r1 - r2 * math.cos(c.dth))))
    return sh, ok & np.isfinite(sh)


def _persistent_near(c, near, persist):
    """持续性近点：命中率高于 persist 且距离中位低于 near 的束。返回 (索引, 中位距离)。"""
    rate = c.valid.mean(axis=0)
    rr = np.where(c.valid, c.r, np.nan)
    with np.errstate(all='ignore'):
        med = np.nanmedian(rr, axis=0)
    sel = (rate > persist) & np.isfinite(med) & (med < near)
    idx = np.flatnonzero(sel)
    return idx, med[idx]


def cmd_compare(args):
    """滤波前后对比。

    ⚠ 两份采集必须**同时**进行（两个 record 进程并行跑），不能先后采。
      2026-08-14 踩过：先后各采 60 s，中间一个 1.5 m 处的东西自己动了，
      于是那几束的命中率 0.86 → 0.09，被当成"滤波器吃掉了真实墙点"。
      场景里只要有任何会动的东西（人、椅子、门），先后采集的差值就不可归因。
    """
    a = Capture(args.raw, args.lidar_yaw)        # /scan
    b = Capture(args.filtered, args.lidar_yaw)   # /scan_filtered
    if a.m != b.m:
        sys.exit(f'束数不一致（{a.m} vs {b.m}）—— 不是同一雷达配置，没法逐束比')
    if abs(math.degrees(a.dth) - math.degrees(b.dth)) > 1e-3:
        sys.exit('角分辨率不一致 —— 两份采集不是同一配置')

    print(f'== B 阶段验收：滤波前后对比 ==  分辨率 {math.degrees(a.dth):.3f}°/束')
    print(f'   raw      {args.raw}：{a.n} 帧')
    print(f'   filtered {args.filtered}：{b.n} 帧')
    na, nb = a.valid.sum(axis=1).mean(), b.valid.sum(axis=1).mean()
    print(f'\n   每帧有效点数：{na:.1f} → {nb:.1f}'
          f'（滤掉 {na - nb:.1f} 点 = {(1 - nb / na) * 100:.1f}%）')

    ok_all = True

    # ── B1/B2：掠射伪点 ──
    sha, oka = _shadow_deg(a)
    shb, okb = _shadow_deg(b)
    ca = (oka & (sha < args.shadow_thresh)).sum(axis=1).mean()
    cb = (okb & (shb < args.shadow_thresh)).sum(axis=1).mean()
    drop = (1 - cb / ca) * 100 if ca > 0 else float('nan')
    print(f'\n── B1/B2 掠射伪点（shadow 角 < {args.shadow_thresh:.0f}°）──')
    print(f'   每帧点对数：{ca:.1f} → {cb:.1f}   **降幅 {drop:.1f}%**（判据 ≥80%）')
    if not (drop >= 80.0):
        print('   ✗ 未达标 —— 先看是不是 min_angle 太宽松，别急着加 neighbors')
        ok_all = False
    else:
        print('   ✓')

    # ── B3a：贴身假障碍 ──
    ia, ma = _persistent_near(a, args.near, args.persist)
    ib, mb = _persistent_near(b, args.near, args.persist)
    print(f'\n── B3a 贴身假障碍（命中率 > {args.persist:.0%} 且距离中位 < {args.near} m）──')
    print(f'   raw 有 {len(ia)} 束，filtered 有 {len(ib)} 束（判据：filtered = 0）')
    for i, m in zip(ib, mb):
        base = math.degrees(float(a.to_base(np.array([a.ang[i]]))[0]))
        print(f'     ✗ 残留：base {base:+7.1f}°  距离中位 {m:.3f} m')
    if len(ib) == 0:
        print('   ✓')
    else:
        ok_all = False

    # ── B3a 不误伤 / B1 不吃真实边缘 ──
    if args.keep_sector:
        print(f'\n── 不误伤：这些扇区的点数不许掉（判据 ≤{args.keep_tol:.0f}%）──')
        a_base = np.degrees(a.to_base(a.ang))
        for spec_str in args.keep_sector:
            try:
                lo, hi = (float(v) for v in spec_str.split(':'))
            except ValueError:
                sys.exit(f'--keep-sector 格式应为 lo:hi（base_link 角度，度），收到 {spec_str!r}')
            sel = (a_base >= lo) & (a_base <= hi) if lo <= hi else (a_base >= lo) | (a_base <= hi)
            ka = a.valid[:, sel].sum(axis=1).mean()
            kb = b.valid[:, sel].sum(axis=1).mean()
            loss = (1 - kb / ka) * 100 if ka > 0 else float('nan')
            mark = '✓' if loss <= args.keep_tol else '✗'
            print(f'   {mark} base [{lo:+.1f}°, {hi:+.1f}°]（{sel.sum()} 束）：'
                  f'每帧 {ka:.1f} → {kb:.1f}，损失 {loss:.1f}%')
            if not (loss <= args.keep_tol):
                ok_all = False
    else:
        print('\n   ⚠ 没给 --keep-sector —— 「不许误伤」这条**没有验**。'
              '\n     把纸盒/门框/桌腿所在的 base_link 角度区间传进来，例如 --keep-sector -20:20')
        ok_all = False

    print(f'\n{"== 全部通过 ==" if ok_all else "== 有未通过项，见上面的 ✗ =="}')


# ─────────────────────────── CLI ───────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--lidar-yaw', type=float, default=DEFAULT_LIDAR_YAW,
                   help='base_link→laser_frame 的 yaw 外参 (rad)')
    sub = p.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('record', help='采集 scan(+odom/imu) 到 npz')
    r.add_argument('--duration', type=float, default=60.0)
    r.add_argument('--out', required=True, help='别放 /tmp —— 开发板重启即失')
    r.add_argument('--scan-topic', default='/scan')
    r.add_argument('--odom-topic', default='/odometry/filtered')
    r.add_argument('--imu-topic', default='/imu_broad/imu')
    r.add_argument('--with-odom', action='store_true', help='A2 需要；A0/A1 不需要')
    r.set_defaults(func=cmd_record)

    v = sub.add_parser('verify', help='A0 按 CAD 尺寸核对立柱')
    v.add_argument('npz')
    v.add_argument('--pillar-r', type=float, default=PILLAR_R_CENTER,
                   help='柱心到底盘圆心的距离 (m)')
    v.add_argument('--pillar-radius', type=float, default=PILLAR_RADIUS, help='柱半径 (m)')
    v.add_argument('--expect', default=PILLAR_BEARINGS,
                   help='立柱在 base_link 下的方位 (deg)，逗号分隔')
    v.add_argument('--tol', type=float, default=2.0, help='核对窗口在半角之外再放宽多少 (deg)')
    v.add_argument('--lower-threshold', type=float, default=0.10,
                   help='laser_filters 的 RangeFilter lower_threshold (m)，用于 F10 判定')
    v.add_argument('--near-thresh', type=float, default=0.15,
                   help='反证用：窗口外距离低于此值的束算意外近点 (m)')
    v.add_argument('--beam-hit', type=float, default=0.05,
                   help='一束要有多少比例的帧测到柱子距离，才算"柱束"（回波是间歇的）')
    v.add_argument('--confirm', type=float, default=0.4,
                   help='柱束数达到 CAD 预期束数的多少倍，才判定该方位核对通过')
    v.set_defaults(func=cmd_verify)

    b = sub.add_parser('noise', help='A1 噪声本底 + 滤波阈值建议')
    b.add_argument('npz')
    b.set_defaults(func=cmd_noise)

    d = sub.add_parser('delay', help='A2/A3 时延 + 帧内畸变')
    d.add_argument('npz')
    d.add_argument('--sector', type=float, default=20.0, help='墙面拟合用的前方半角 (deg)')
    d.add_argument('--max-resid', type=float, default=0.02, help='直线拟合 RMS 残差上限 (m)')
    d.add_argument('--omega-min', type=float, default=0.1, help='判定"在转"的角速度门限 (rad/s)')
    d.add_argument('--max-jump', type=float, default=15.0,
                   help='帧间墙面法向跳变上限 (deg)，超过判为换了一面墙')
    d.add_argument('--deskewed', action='store_true',
                   help='数据来自已做 deskew 的链路（C2 之后）；置位则不做 t_eff 修正')
    d.set_defaults(func=cmd_delay)

    t = sub.add_parser('tshift', help='时间戳偏移标定（取代 delay，A2/C1 的权威口径）')
    t.add_argument('npz', help='摆动采集（record --with-odom）')
    t.add_argument('--half', type=int, default=20, help='窗半宽（束），默认 20')
    t.add_argument('--step', type=int, default=20, help='窗中心步长（束），默认 20')
    t.add_argument('--max-resid', type=float, default=0.015, help='直线拟合 RMS 残差上限 (m)')
    t.add_argument('--min-frames', type=int, default=120, help='一个窗至少要拟出多少帧')
    t.add_argument('--max-rms', type=float, default=2.6,
                   help='采用一个窗的残差上限 (deg)。近距/杂乱场景要放宽 —— '
                        '先看 Δ 是否已排成直线，别把好数据判成没数据')
    t.add_argument('--max-jump', type=float, default=15.0,
                   help='帧间法向跳变上限 (deg)，超了说明窗里换了物体')
    t.add_argument('--search', type=float, default=0.30, help='Δ 搜索范围 ±s (秒)')
    t.set_defaults(func=cmd_tshift)

    k = sub.add_parser('compare', help='B 阶段验收：滤波前后对比（两份同场景静止采集）')
    k.add_argument('raw', help='/scan 的采集')
    k.add_argument('filtered', help='/scan_filtered 的采集')
    k.add_argument('--shadow-thresh', type=float, default=10.0,
                   help='判为掠射伪点的 shadow 角上限 (deg)，默认 10')
    k.add_argument('--near', type=float, default=0.25,
                   help='贴身假障碍的距离上限 (m)，默认 0.25')
    k.add_argument('--persist', type=float, default=0.5,
                   help='算「持续性」的命中率下限，默认 0.5')
    k.add_argument('--keep-sector', action='append', metavar='LO:HI',
                   help='不许误伤的 base_link 角度区间 (deg)，可重复给。'
                        '⚠ 负角度必须用等号形式，否则 argparse 会把它当成选项：'
                        '--keep-sector=-20:20 --keep-sector=80:110')
    k.add_argument('--keep-tol', type=float, default=5.0,
                   help='上述区间允许的点数损失上限 (%%)，默认 5')
    k.set_defaults(func=cmd_compare)

    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
