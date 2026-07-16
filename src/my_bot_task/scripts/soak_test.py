#!/usr/bin/env python3
"""M5：长时间无人值守压测。

在 M2 的动态办公室世界里，伪随机循环发起召唤（模拟真实使用的不规律呼叫节奏，
偶尔提前"我倒完了"取消掉），同时定期用 /set_pose 硬灌一次相对当前位置的小幅假
位姿扰动（模拟一次瞬时传感器坏读数），观察系统在没有任何主动定位自愈机制介入的
情况下，能不能靠 SLAM 自身持续的 scan matching organic 收敛回去。

全程记录 /task/status、/nav_watchdog/status 的每一条【状态变化】（不是每一条
消息——那些是 latched/1Hz 轮询，大部分内容没变，记全量只会让日志爆炸不好看）到
JSONL，跑完输出量化汇总：MTBF、故障注入次数、scan_map_alignment 时间序列统计。

scan_map_alignment 是定位正确性的独立验证信号：拿静态地图（.pgm + .yaml，SLAM
建图时产出的那份，不是实时地图）自己算"当前激光扫描点有多少落在地图已知障碍物
附近"的对齐分数，同时吃到 EKF（odom→base_footprint）和 SLAM 的实时修正
（map→odom）两段 TF。⚠️ 不要只信 nav_watchdog 自己的 OK/DIVERGED——那个信号只看
EKF 有没有再跳变，跳变停了就报 OK，不代表位姿真的对；这里必须用一个不经过自报
信号中转的量，才能验证"真的收敛了"还是"平滑地停在了错误的地方"。

用法：
    ros2 run my_bot_task soak_test.py --ros-args \
        -p use_sim_time:=true -p duration_sec:=1800.0
"""

import json
import math
import os
import random
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

MAPS_ROOT = os.path.expanduser('~/.maps')


def resolve_map_name(explicit: str) -> str:
    if explicit:
        return explicit
    try:
        with open(os.path.join(MAPS_ROOT, 'current_map')) as f:
            return f.read().strip()
    except OSError:
        return ''


class SoakTest(Node):
    def __init__(self):
        super().__init__('soak_test')

        self.declare_parameter('map_name', '')
        self.declare_parameter('duration_sec', 1800.0)
        self.declare_parameter('min_call_interval_sec', 8.0)
        self.declare_parameter('max_call_interval_sec', 40.0)
        self.declare_parameter('early_cancel_prob', 0.3)
        self.declare_parameter('early_cancel_min_sec', 3.0)
        self.declare_parameter('early_cancel_max_sec', 15.0)
        # 每隔这么久注一次定位扰动，主动考验系统在没有主动自愈机制下的容错边界
        # ——不是等它自然发生。
        # ⚠️ 必须是【相对当前位置】的小幅扰动（模拟一次瞬时传感器抽风/坏读数），
        # 不能是整栋楼内随机传送——那是"绑架机器人"式的全局重定位问题，slam_toolbox
        # 的增量定位模式设计上处理不了，注一个它注定救不回来的故障没有测试意义。
        self.declare_parameter('fault_inject_interval_sec', 300.0)
        self.declare_parameter('fault_inject_offset_m', 0.4)
        # scan_map_alignment：独立于 nav_watchdog 的地图对齐分数，见类头部说明
        self.declare_parameter('alignment_check_period_sec', 3.0)
        self.declare_parameter('alignment_tol_m', 0.15)

        self.duration = float(self.get_parameter('duration_sec').value)
        self.min_interval = float(self.get_parameter('min_call_interval_sec').value)
        self.max_interval = float(self.get_parameter('max_call_interval_sec').value)
        self.early_cancel_prob = float(self.get_parameter('early_cancel_prob').value)
        self.early_cancel_min = float(self.get_parameter('early_cancel_min_sec').value)
        self.early_cancel_max = float(self.get_parameter('early_cancel_max_sec').value)
        self.fault_interval = float(self.get_parameter('fault_inject_interval_sec').value)
        self.fault_offset = float(self.get_parameter('fault_inject_offset_m').value)

        self.points = self._load_topology()
        self.rooms = [n for n in self.points if n.upper() != 'HOME']
        if not self.rooms:
            raise SystemExit('拓扑里没有可用的房间点 —— 先在 app 上标点')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, '/goal_pose/cancel', 10)
        self.set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose', 10)

        self.create_subscription(String, '/task/status', self._on_task_status, latched)
        self.create_subscription(String, '/nav_watchdog/status', self._on_nav_wd, latched)
        self.current_pose = None   # (x, y, yaw)，来自 /odometry/filtered —— 扰动要基于这个算
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)

        self.map_data = self._load_map()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_alignment_check = -1e9
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan, 10)

        name = resolve_map_name(str(self.get_parameter('map_name').value))
        self.log_dir = os.path.join(MAPS_ROOT, name) if name else '/tmp'
        os.makedirs(self.log_dir, exist_ok=True)
        ts = int(time.time())
        self.log_path = os.path.join(self.log_dir, f'soak_{ts}.jsonl')
        self.summary_path = os.path.join(self.log_dir, f'soak_{ts}_summary.json')
        self.log_file = open(self.log_path, 'a')

        self.last_task_state = ''
        self.last_nav_wd_status = ''

        self.calls_sent = 0
        self.arrivals = 0
        self.stuck_events = 0
        self.faults_injected = 0
        self.alignment_scores = []   # [(t, score), ...] —— 见 _on_scan

        # 非阻塞的"定时任务"——全部靠在主循环里比较 elapsed 触发，不用 time.sleep()
        # 卡住主循环（那样会漏接/延后处理这段时间里进来的状态消息）。
        self.pending_cancel_at = None
        self.start_time = None   # run() 里才真正起算，构造函数阶段时钟可能还没同步

        self.get_logger().info(
            f'压测就绪：时长 {self.duration:.0f}s，房间 {len(self.rooms)} 个，'
            f'日志 {self.log_path}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_topology(self):
        name = resolve_map_name(str(self.get_parameter('map_name').value))
        if not name:
            return {}
        path = os.path.join(MAPS_ROOT, name, f'{name}.topology')
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return {}
        return {p['name']: p for p in data.get('points', []) if p.get('name')}

    def _load_map(self):
        """读 SLAM 建图时产出的静态 .pgm/.yaml——不是实时地图，是地面真值参照。"""
        name = resolve_map_name(str(self.get_parameter('map_name').value))
        if not name:
            return None
        yaml_path = os.path.join(MAPS_ROOT, name, f'{name}.yaml')
        try:
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            img_path = os.path.join(MAPS_ROOT, name, meta['image'])
            arr = np.array(Image.open(img_path).convert('L'))
            # map_server 的换算：occ_prob = (255-pixel)/255；occ_prob > occupied_thresh
            # 才算占据。阈值优先读 yaml 里的，没有就用 nav2 默认 0.65。
            occ_thresh = float(meta.get('occupied_thresh', 0.65))
            pixel_thresh = 255.0 * (1.0 - occ_thresh)
            occupied = arr < pixel_thresh
            self.get_logger().info(
                f'已加载静态地图 {img_path}（{arr.shape[1]}x{arr.shape[0]}，'
                f'占据阈值像素<{pixel_thresh:.0f}），scan_map_alignment 可用')
            return {
                'occupied': occupied,
                'resolution': float(meta['resolution']),
                'origin': (float(meta['origin'][0]), float(meta['origin'][1])),
                'height': arr.shape[0],
                'width': arr.shape[1],
            }
        except Exception as exc:  # noqa: BLE001 - 地图加载失败不该拖垮整个压测
            self.get_logger().warn(
                f'加载静态地图失败（{exc}）—— scan_map_alignment 这部分不可用，'
                f'其余压测照常跑')
            return None

    def _world_to_pixel(self, x, y):
        m = self.map_data
        col = int((x - m['origin'][0]) / m['resolution'])
        row = m['height'] - 1 - int((y - m['origin'][1]) / m['resolution'])
        return row, col

    def _on_scan(self, msg: LaserScan):
        if self.map_data is None or self.start_time is None:
            return
        now = self._now()
        if now - self.last_alignment_check < float(
                self.get_parameter('alignment_check_period_sec').value):
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except tf2_ros.TransformException:
            return
        self.last_alignment_check = now

        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        occ = self.map_data['occupied']
        h, w = occ.shape
        tol_cells = max(1, int(round(
            float(self.get_parameter('alignment_tol_m').value) / self.map_data['resolution'])))

        hits, total = 0, 0
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                lx, ly = r * math.cos(angle), r * math.sin(angle)
                wx = tx + lx * cos_y - ly * sin_y
                wy = ty + lx * sin_y + ly * cos_y
                row, col = self._world_to_pixel(wx, wy)
                total += 1
                r0, r1 = max(0, row - tol_cells), min(h, row + tol_cells + 1)
                c0, c1 = max(0, col - tol_cells), min(w, col + tol_cells + 1)
                if r0 < r1 and c0 < c1 and occ[r0:r1, c0:c1].any():
                    hits += 1
            angle += msg.angle_increment

        if total > 0:
            score = hits / total
            self.alignment_scores.append((round(now - self.start_time, 2), round(score, 3)))
            self._log_event('scan_map_alignment', {'score': round(score, 3), 'n_points': total})

    def _log_event(self, kind, payload):
        rec = {'t': round(self._now() - self.start_time, 2), 'kind': kind, **payload}
        self.log_file.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.log_file.flush()

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = data.get('state', '')
        if self.start_time is not None and state != self.last_task_state:
            self._log_event('task_state', {
                'from': self.last_task_state, 'to': state,
                'current': data.get('current'), 'last_error': data.get('last_error', '')})
            if state == 'WAITING':
                self.arrivals += 1
        self.last_task_state = state

    def _on_nav_wd(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        status = data.get('status', '')
        if self.start_time is not None and status != self.last_nav_wd_status:
            self._log_event('nav_watchdog', data)
            if status == 'STUCK':
                self.stuck_events += 1
        self.last_nav_wd_status = status

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = (p.x, p.y, yaw)

    def _send_call(self, elapsed):
        name = random.choice(self.rooms)
        point = self.points[name]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point['x'])
        msg.pose.position.y = float(point['y'])
        theta = float(point.get('theta', 0.0))
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.goal_pub.publish(msg)
        self.calls_sent += 1
        self._log_event('call', {'room': name})

        if random.random() < self.early_cancel_prob:
            self.pending_cancel_at = elapsed + random.uniform(
                self.early_cancel_min, self.early_cancel_max)

    def _inject_fault(self):
        if self.current_pose is None:
            self.get_logger().warn('还没收到 /odometry/filtered，这次跳过注入')
            return

        cx, cy, cyaw = self.current_pose
        # ⚠️ 相对当前位置的小幅扰动，不是绝对坐标——模拟一次瞬时传感器坏读数，
        # 不是"瞬间传送到房子另一头"那种绑架机器人场景（这套系统没打算能从那种
        # 情况自愈，见类头部注释）。
        dx = random.uniform(-self.fault_offset, self.fault_offset)
        dy = random.uniform(-self.fault_offset, self.fault_offset)
        dyaw = random.uniform(-0.2, 0.2)   # ±约11°的小幅转向扰动
        new_x, new_y, new_yaw = cx + dx, cy + dy, cyaw + dyaw

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = new_x
        msg.pose.pose.position.y = new_y
        msg.pose.pose.orientation.z = math.sin(new_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(new_yaw / 2.0)
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.set_pose_pub.publish(msg)
        self.faults_injected += 1
        self._log_event('fault_inject', {
            'from': [round(cx, 2), round(cy, 2)], 'delta': [round(dx, 2), round(dy, 2)]})

    def run(self):
        # 用 sim time 时，get_clock().now() 在收到第一条 /clock 消息之前一直是 0——
        # 如果 start_time 在这之前采样，之后仿真时钟一到位，elapsed 会瞬间跳成
        # "当前真实仿真时刻 - 0"，离谱地大（在一个已经跑了一阵子的仿真上启动本
        # 脚本时会踩到；刚启动的全新仿真上巧合地不明显，容易被漏掉——这次是靠
        # 60s 冒烟测试才抓出来的）。先空转到收到至少一条 /clock 再起算。
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.start_time = self._now()
        next_call = 0.0
        next_fault = self.fault_interval

        while True:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = self._now() - self.start_time
            if elapsed >= self.duration:
                break

            if elapsed >= next_call:
                self._send_call(elapsed)
                next_call = elapsed + random.uniform(self.min_interval, self.max_interval)

            if elapsed >= next_fault:
                self._inject_fault()
                next_fault = elapsed + self.fault_interval

            if self.pending_cancel_at is not None and elapsed >= self.pending_cancel_at:
                self.cancel_pub.publish(Empty())
                self._log_event('early_cancel', {})
                self.pending_cancel_at = None

        self._report()

    def _report(self):
        elapsed = self._now() - self.start_time
        mtbf = elapsed / self.stuck_events if self.stuck_events else None

        scores = [s for _, s in self.alignment_scores]
        # 低于这个分数就当作"明显没对齐"——0.15m 容差、静态地图，正常对齐时
        # 大多数扫描点应该落在已知障碍物附近；貌似离谱的低分才值得警惕。
        low_score_threshold = 0.3
        low_score_samples = sum(1 for s in scores if s < low_score_threshold)

        summary = {
            'duration_sec': round(elapsed, 1),
            'calls_sent': self.calls_sent,
            'arrivals': self.arrivals,
            'stuck_events': self.stuck_events,
            'faults_injected': self.faults_injected,
            'mtbf_sec': round(mtbf, 1) if mtbf else None,
            # 独立验证信号——见类头部 scan_map_alignment 说明。直接衡量注入扰动后
            # 位姿有没有真的收敛回去，不经过任何自报信号中转。
            'scan_map_alignment_samples': len(scores),
            'scan_map_alignment_avg': round(sum(scores) / len(scores), 3) if scores else None,
            'scan_map_alignment_min': round(min(scores), 3) if scores else None,
            'scan_map_alignment_low_score_count': low_score_samples,
        }

        print('\n==================== M5 压测汇总 ====================')
        for k, v in summary.items():
            print(f'{k}: {v}')

        with open(self.summary_path, 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f'\n事件日志: {self.log_path}')
        print(f'汇总: {self.summary_path}')

        self.log_file.close()


def main():
    rclpy.init()
    node = SoakTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
