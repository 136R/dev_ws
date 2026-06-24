#!/usr/bin/env python3
"""根据 ROS_Flutter_Gui_App 的"画笔"编辑结果生成 Nav2 Keepout 滤波掩码。

方案一（不改 app 源码）。app 后端并不持续把编辑后的地图发到 ROS 话题，
而是把"原图 + 画笔"持久化到磁盘 PGM（~/.maps/<map>/map.pgm），前端走 HTTP 瓦片。
因此本节点：

  - clean 基准 : slam_toolbox 持续发布的 /map（永远不含用户画笔）
  - edited 图  : 磁盘 PGM 文件（含用户画笔），轮询 mtime，变更即重载
  - keepout    : 在 edited 中"占据"、而在 clean 中并非占据的栅格
                 = 用户新增的禁行区 → 发布为独立掩码 /keepout_filter_mask

掩码语义：禁行格 = 100，其余 = -1(未知)。用未知而非 0(空闲)，
保证 KeepoutFilter 只"叠加"禁行区、绝不擦除静态层的真实障碍。
"""

import os

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid

UNKNOWN = -1
KEEPOUT = 100


def latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _read_pgm_tokens(data: bytes):
    """读取 P5 PGM 头部 token（跳过 # 注释），返回 (magic, w, h, maxval, pixel_offset)。"""
    tokens = []
    i = 0
    n = len(data)
    while len(tokens) < 4 and i < n:
        c = data[i:i + 1]
        if c in b' \t\r\n':
            i += 1
            continue
        if c == b'#':  # 注释到行尾
            while i < n and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < n and data[j:j + 1] not in b' \t\r\n':
            j += 1
        tokens.append(data[i:j])
        i = j
    # token 之后紧跟一个空白分隔符，再就是像素数据
    if i < n and data[i:i + 1] in b' \t\r\n':
        i += 1
    magic = tokens[0]
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    return magic, w, h, maxval, i


def load_pgm_as_occupancy(yaml_path: str):
    """按 map_server 约定把 PGM 转成 (info_dict, occ_array[H,W])，row0=origin(底行)。"""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    pgm_path = meta['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(map_dir, pgm_path)

    with open(pgm_path, 'rb') as f:
        raw = f.read()
    magic, w, h, maxval, off = _read_pgm_tokens(raw)
    if magic != b'P5':
        raise ValueError(f'仅支持二进制 PGM(P5)，得到 {magic!r}')
    img = np.frombuffer(raw[off:off + w * h], dtype=np.uint8).reshape(h, w)

    negate = int(meta.get('negate', 0))
    occ_thr = float(meta.get('occupied_thresh', 0.65))
    free_thr = float(meta.get('free_thresh', 0.25))

    val = img.astype(np.float64)
    occ = (val / maxval) if negate else ((maxval - val) / maxval)
    out = np.full((h, w), UNKNOWN, dtype=np.int16)
    out[occ > occ_thr] = 100
    out[occ < free_thr] = 0
    # PGM 顶行在前；OccupancyGrid data 从底行(origin)开始 → 上下翻转
    out = np.flipud(out)
    info = {
        'resolution': float(meta['resolution']),
        'width': w,
        'height': h,
        'origin_x': float(meta['origin'][0]),
        'origin_y': float(meta['origin'][1]),
        'pgm_path': pgm_path,
    }
    return info, out


class KeepoutMaskDiff(Node):
    def __init__(self):
        super().__init__('keepout_mask_diff')

        self.declare_parameter('clean_map_topic', '/map')
        self.declare_parameter('mask_topic', '/keepout_filter_mask')
        # 一次性把 clean 图转发到该话题（latched），供 app 后端订阅，
        # 让 app 只镜像一次地图、不再每 2s 覆盖磁盘 PGM 抹掉画笔。空串=关闭转发。
        self.declare_parameter('gui_map_topic', '/map_gui')
        self.declare_parameter(
            'edited_map_yaml',
            os.path.expanduser('~/.maps/map/map.yaml'))
        # 自动跟随 app 当前地图：读 <maps_root>/current_map 决定监视哪张图，
        # 这样在 app 里切换/编辑任意地图都能生效（无需重启节点）。
        self.declare_parameter('follow_current_map', True)
        self.declare_parameter('maps_root', os.path.expanduser('~/.maps'))
        self.declare_parameter('poll_period_sec', 1.0)
        self.declare_parameter('occupied_threshold', 65)

        self.clean_topic = self.get_parameter('clean_map_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value
        self.gui_map_topic = self.get_parameter('gui_map_topic').value
        self.edited_yaml = os.path.expanduser(
            self.get_parameter('edited_map_yaml').value)
        self.follow_current = bool(self.get_parameter('follow_current_map').value)
        self.maps_root = os.path.expanduser(self.get_parameter('maps_root').value)
        self.occ_thr = int(self.get_parameter('occupied_threshold').value)
        poll = float(self.get_parameter('poll_period_sec').value)

        self.clean_map: OccupancyGrid | None = None
        self.gui_relayed = False
        self.active_yaml = self.edited_yaml
        self.edited_info: dict | None = None
        self.edited_arr: np.ndarray | None = None
        self.edited_mtime: float | None = None
        self.last_published: np.ndarray | None = None

        self.mask_pub = self.create_publisher(
            OccupancyGrid, self.mask_topic, latched_qos())
        self.gui_map_pub = None
        if self.gui_map_topic:
            self.gui_map_pub = self.create_publisher(
                OccupancyGrid, self.gui_map_topic, latched_qos())
        self.create_subscription(
            OccupancyGrid, self.clean_topic, self._on_clean, latched_qos())
        self.create_timer(poll, self._poll_edited)

        edited_src = (f'follow current_map @ {self.maps_root}'
                      if self.follow_current else self.edited_yaml)
        self.get_logger().info(
            f'keepout_mask_diff 启动: clean={self.clean_topic} '
            f'edited(PGM)={edited_src} -> mask={self.mask_topic} '
            f'(occupied_threshold={self.occ_thr}, gui_relay={self.gui_map_topic or "off"})')

    def _on_clean(self, msg: OccupancyGrid):
        # clean 基准只取第一帧并冻结：localization 模式地图静态，避免基准漂移。
        if self.clean_map is None:
            self.clean_map = msg
            # 一次性把 clean 图转发给 app（latched），之后不再发 → app 不再覆盖磁盘
            if self.gui_map_pub is not None and not self.gui_relayed:
                self.gui_map_pub.publish(msg)
                self.gui_relayed = True
                self.get_logger().info(
                    f'已一次性转发 clean 图到 {self.gui_map_topic}（供 app 订阅）')
        self._recompute()

    def _resolve_edited_yaml(self) -> str:
        """跟随 app 当前地图：读 <maps_root>/current_map，返回 <root>/<name>/<name>.yaml。"""
        if not self.follow_current:
            return self.edited_yaml
        try:
            with open(os.path.join(self.maps_root, 'current_map'), 'r') as f:
                name = f.read().strip()
        except OSError:
            name = ''
        if not name:
            return self.edited_yaml
        return os.path.join(self.maps_root, name, name + '.yaml')

    def _poll_edited(self):
        yaml_path = self._resolve_edited_yaml()
        if yaml_path != self.active_yaml:
            self.get_logger().info(f'切换到当前地图: {yaml_path}')
            self.active_yaml = yaml_path
            self.edited_mtime = None      # 强制重载
            self.last_published = None     # 强制重发
        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            map_dir = os.path.dirname(os.path.abspath(yaml_path))
            pgm = meta['image']
            pgm = pgm if os.path.isabs(pgm) else os.path.join(map_dir, pgm)
            mtime = os.path.getmtime(pgm)
        except (FileNotFoundError, KeyError, OSError):
            return
        if self.edited_mtime is not None and mtime == self.edited_mtime:
            return
        try:
            self.edited_info, self.edited_arr = load_pgm_as_occupancy(yaml_path)
            self.edited_mtime = mtime
            self.get_logger().info(f'已加载编辑地图 PGM (mtime 变更): {pgm}')
            self._recompute()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'加载编辑 PGM 失败: {e}')

    def _clean_resampled(self) -> np.ndarray:
        """把 clean(/map) 按世界坐标重采样到 edited PGM 栅格上（忽略 origin 旋转）。"""
        clean = self.clean_map
        info = self.edited_info
        cl = np.array(clean.data, dtype=np.int16).reshape(
            clean.info.height, clean.info.width)

        He, We = info['height'], info['width']
        res_e, ox_e, oy_e = info['resolution'], info['origin_x'], info['origin_y']
        res_c = clean.info.resolution
        ox_c, oy_c = clean.info.origin.position.x, clean.info.origin.position.y
        Hc, Wc = clean.info.height, clean.info.width

        same = (He == Hc and We == Wc
                and abs(res_e - res_c) < 1e-9
                and abs(ox_e - ox_c) < 1e-3 and abs(oy_e - oy_c) < 1e-3)
        if same:
            return cl

        xs = ox_e + (np.arange(We) + 0.5) * res_e
        ys = oy_e + (np.arange(He) + 0.5) * res_e
        cj = np.floor((xs - ox_c) / res_c).astype(np.int64)
        ci = np.floor((ys - oy_c) / res_c).astype(np.int64)
        vj = (cj >= 0) & (cj < Wc)
        vi = (ci >= 0) & (ci < Hc)
        out = np.full((He, We), UNKNOWN, dtype=np.int16)
        CI, CJ = np.meshgrid(ci, cj, indexing='ij')
        V = vi[:, None] & vj[None, :]
        out[V] = cl[CI[V], CJ[V]]
        return out

    def _recompute(self):
        if self.clean_map is None or self.edited_arr is None:
            return

        ed = self.edited_arr
        cl = self._clean_resampled()
        keepout = (ed >= self.occ_thr) & (cl < self.occ_thr)

        mask = np.full(ed.shape, UNKNOWN, dtype=np.int8)
        mask[keepout] = KEEPOUT
        flat = mask.reshape(-1)

        if self.last_published is not None and np.array_equal(
                flat, self.last_published):
            return
        self.last_published = flat.copy()

        info = self.edited_info
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.clean_map.header.frame_id or 'map'
        out.info.resolution = info['resolution']
        out.info.width = info['width']
        out.info.height = info['height']
        out.info.origin.position.x = info['origin_x']
        out.info.origin.position.y = info['origin_y']
        out.info.origin.orientation.w = 1.0
        out.data = flat.tolist()
        self.mask_pub.publish(out)
        self.get_logger().info(
            f'已发布 keepout 掩码: {int(keepout.sum())} 个禁行栅格 '
            f"({info['width']}x{info['height']})")


def main():
    rclpy.init()
    node = KeepoutMaskDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
