#!/usr/bin/env python3
"""仿真建图质量：拿 slam 建的占据栅格和 sdf 真实墙面比。

要求地图从原点建（map = world）。指标（都按格子中心算，5cm 一格时完美地图也有 ~2.5cm 的格子误差，
先用 --synthetic 看完美栅格的"地板"再比）：
  occ_out p50/p95/max   占据格中心离真墙面多远（墙外为正）——墙画厚、画偏
  occ_out_al_p95        扣掉下面 align 的整图偏移后再算 —— 只剩"画厚"
  ghost                 占据格离任何墙 > 10cm 的个数 —— 重影 / 噪点
  recall                看得见的墙面（外侧 15cm 内有空闲格）上，6cm 内有占据格的比例 —— 墙漏画
  false_occ             真空闲区（离墙 > 10cm）被标成占据的比例
  align dx/dy/dθ        整张图相对世界最可能的平移 / 旋转（网格搜索 ±10cm、±2°）

用法：
  ros2 run my_bot_nav map_eval.py --topic                                  # 读 /map（建图或定位中）
  ros2 run my_bot_nav map_eval.py --yaml ~/.maps/my_world_40/my_world_40.yaml
  ros2 run my_bot_nav map_eval.py --synthetic                              # 自检：sdf 栅格化的完美地图
  ros2 run my_bot_nav map_eval.py --synthetic --shift 0.05,0,0             # 自检：人为平移 5cm 能不能测出来
  可加 --world <sdf>（默认 my_world_40）、--out <json>
"""
import argparse
import json
import math
import os
import re

import numpy as np

RES_DEFAULT = 0.05


def load_walls(path):
    """sdf 墙体 → 轴对齐矩形 (xmin, xmax, ymin, ymax)。同 nav_route_test.py：外墙 + 0/90° 的 obstacle_* 墙。"""
    s = open(path).read()
    walls = [(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)
             for x, y, sx, sy in [(2, 0, 0.1, 4), (0, 2, 4, 0.1), (-2, 0, 0.1, 4), (0, -2, 4, 0.1)]]
    for m in re.finditer(r'<link name="(obstacle_[^"]+)">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', s, re.S):
        x, y, _, _, _, yaw = map(float, m.group(2).split())
        sx, sy, _ = map(float, m.group(3).split())
        if abs(abs(yaw) - 1.5708) < 0.01:
            sx, sy = sy, sx
        walls.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return np.array(walls)


def sdist(walls, x, y):
    """点集到最近墙面的有符号距离（墙内为负），向量化。"""
    best = np.full(x.shape, 9.0)
    for a, b, c, d in walls:
        dx = np.maximum(np.maximum(a - x, 0), x - b)
        dy = np.maximum(np.maximum(c - y, 0), y - d)
        out = np.hypot(dx, dy)
        inside = -np.minimum(np.minimum(x - a, b - x), np.minimum(y - c, d - y))
        best = np.minimum(best, np.where((dx == 0) & (dy == 0), inside, out))
    return best


def wall_surface_points(walls, step=0.025):
    """每面墙外廓上每 2.5cm 一个点，带朝外的法向。"""
    pts = []
    for a, b, c, d in walls:
        for x in np.arange(a, b + 1e-9, step):
            pts += [(x, c, 0, -1), (x, d, 0, 1)]
        for y in np.arange(c, d + 1e-9, step):
            pts += [(a, y, -1, 0), (b, y, 1, 0)]
    return np.array(pts)


def grid_from_yaml(path):
    """map_saver 的 pgm/yaml → (状态网格 1=占据 0=空闲 -1=未知, 分辨率, 原点 x, y)。行 0 = 最小 y。"""
    meta = dict(re.findall(r'^(\w+):\s*(.+)$', open(path).read(), re.M))
    img = os.path.join(os.path.dirname(path), meta['image'].strip())
    raw = open(img, 'rb').read()
    tok = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', raw)
    w, h = int(tok.group(1)), int(tok.group(2))
    px = np.frombuffer(raw[tok.end():tok.end() + w * h], np.uint8).reshape(h, w)[::-1].astype(float)
    p = (255 - px) / 255.0 if int(meta.get('negate', '0')) == 0 else px / 255.0
    g = np.full(p.shape, -1, int)
    g[p > float(meta['occupied_thresh'])] = 1
    g[p < float(meta['free_thresh'])] = 0
    ox, oy = [float(v) for v in meta['origin'].strip('[] ').split(',')[:2]]
    return g, float(meta['resolution']), ox, oy


def grid_from_topic():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    n = rclpy.create_node('map_eval')
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/map', got.append, qos)
    while not got:
        rclpy.spin_once(n, timeout_sec=0.5)
    m = got[-1]
    d = np.array(m.data, int).reshape(m.info.height, m.info.width)
    g = np.full(d.shape, -1, int)
    g[d >= 65] = 1
    g[(d >= 0) & (d <= 25)] = 0
    rclpy.shutdown()
    return g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y


def grid_synthetic(walls, res=RES_DEFAULT):
    """完美地图：格子和墙有交集 = 占据，其余在外墙内 = 空闲。原点同 slam 习惯落在 -1.97。"""
    ox = oy = -1.97 - 2 * res
    n = int(4.2 / res) + 4
    xs = ox + (np.arange(n) + 0.5) * res
    X, Y = np.meshgrid(xs, xs)
    d = sdist(walls, X, Y)
    g = np.where(d <= res / 2, 1, 0)                          # 中心离墙 ≤ 半格 ≈ 格子碰到墙
    g[(np.abs(X) > 2.05) | (np.abs(Y) > 2.05)] = -1
    return g, res, ox, oy


def evaluate(g, res, ox, oy, walls, shift=(0.0, 0.0, 0.0)):
    iy, ix = np.nonzero(g == 1)
    if len(ix) == 0:
        return dict(n_occ=0)                                   # 占据阈值太高时整张图没有墙
    x, y = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
    sx, sy, sth = shift                                        # 自检：把整张图绕原点转 sth 再平移
    c, s = math.cos(sth), math.sin(sth)
    x, y = c * x - s * y + sx, s * x + c * y + sy
    d = sdist(walls, x, y)
    # 空闲格
    fy, fx = np.nonzero(g == 0)
    fxw, fyw = ox + (fx + 0.5) * res, oy + (fy + 0.5) * res
    fxw, fyw = c * fxw - s * fyw + sx, s * fxw + c * fyw + sy
    fd = sdist(walls, fxw, fyw)
    # 墙面召回：外侧 15cm 内有空闲格才算"看得见"
    sp = wall_surface_points(walls)
    sp = sp[sdist(walls, sp[:, 0] + sp[:, 2] * 0.003, sp[:, 1] + sp[:, 3] * 0.003) > 0]   # 去掉和别的墙贴着的面
    def near(px, py, qx, qy, r):
        if len(qx) == 0:
            return np.zeros(len(px), bool)
        out = np.zeros(len(px), bool)
        for i in range(0, len(px), 400):
            dd = (px[i:i + 400, None] - qx[None]) ** 2 + (py[i:i + 400, None] - qy[None]) ** 2
            out[i:i + 400] = (dd < r * r).any(1)
        return out
    vis = near(sp[:, 0] + sp[:, 2] * 0.15, sp[:, 1] + sp[:, 3] * 0.15, fxw, fyw, 0.075)
    hit = near(sp[:, 0], sp[:, 1], x, y, 0.06)
    # 整图对齐：最小化占据格到墙面的平均 |距离|
    best = (9, 0, 0, 0)
    near_w = np.abs(d) < 0.12
    bx, by = x[near_w], y[near_w]
    for th in np.radians(np.arange(-2, 2.01, 0.25)):
        ct, st = math.cos(th), math.sin(th)
        rx, ry = ct * bx - st * by, st * bx + ct * by
        for ddx in np.arange(-0.10, 0.101, 0.01):
            for ddy in np.arange(-0.10, 0.101, 0.01):
                v = np.mean(np.abs(sdist(walls, rx + ddx, ry + ddy))) if len(bx) else 9
                if v < best[0]:
                    best = (v, ddx, ddy, th)
    pos = np.maximum(d, 0)
    # 扣掉整图偏移后墙还厚多少（区分"整体偏了"和"画厚了"）
    ct, st = math.cos(best[3]), math.sin(best[3])
    pos_al = np.maximum(sdist(walls, ct * x - st * y + best[1], st * x + ct * y + best[2]), 0)
    r = lambda v: round(float(v), 2)
    return dict(
        n_occ=int(len(d)),
        occ_out_p50=r(np.percentile(pos, 50) * 100), occ_out_p95=r(np.percentile(pos, 95) * 100),
        occ_out_max=r(pos.max() * 100), occ_out_al_p95=r(np.percentile(pos_al, 95) * 100),
        ghost=int((d > 0.10).sum()),
        recall=r(hit[vis].mean()) if vis.any() else None, wall_pts_seen=int(vis.sum()),
        false_occ=round(float((d > 0.10).sum()) / max(1, int((fd > 0.10).sum() + (d > 0.10).sum())), 4),
        align_dx=r(-best[1] * 100), align_dy=r(-best[2] * 100), align_dth=r(-math.degrees(best[3])))


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--topic', action='store_true')
    src.add_argument('--yaml')
    src.add_argument('--synthetic', action='store_true')
    ap.add_argument('--world', default=None)
    ap.add_argument('--shift', default='0,0,0', help='自检用：dx,dy,dθ(度)')
    ap.add_argument('--out')
    ap.add_argument('--save', help='把读到的栅格另存成 <路径>.pgm/.yaml（--topic 时用）')
    a = ap.parse_args()
    if a.world is None:
        from ament_index_python.packages import get_package_share_directory
        a.world = os.path.join(get_package_share_directory('my_bot'), 'worlds', 'my_world_40.sdf')
    walls = load_walls(a.world)
    if a.topic:
        g, res, ox, oy = grid_from_topic()
    elif a.yaml:
        g, res, ox, oy = grid_from_yaml(os.path.expanduser(a.yaml))
    else:
        g, res, ox, oy = grid_synthetic(walls)
    if a.save:
        img = np.full(g.shape, 205, np.uint8)
        img[g == 0], img[g == 1] = 254, 0
        open(a.save + '.pgm', 'wb').write(b'P5\n%d %d\n255\n' % (g.shape[1], g.shape[0]) + img[::-1].tobytes())
        open(a.save + '.yaml', 'w').write(f'image: {os.path.basename(a.save)}.pgm\nmode: trinary\nresolution: {res}\n'
                                          f'origin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    dx, dy, dth = [float(v) for v in a.shift.split(',')]
    m = evaluate(g, res, ox, oy, walls, (dx, dy, math.radians(dth)))
    print(json.dumps(m, ensure_ascii=False))
    if a.out:
        json.dump(m, open(a.out, 'w'))


if __name__ == '__main__':
    main()
