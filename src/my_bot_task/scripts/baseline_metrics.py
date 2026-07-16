#!/usr/bin/env python3
"""动态环境导航基线体检（M2）。

不改任何导航代码，只是拿现成的"召唤"接口（/goal_pose）按拓扑点名字挨个跑一遍，
边跑边记：

    trip_sec          单次召唤从 NAVIGATING 到 WAITING 的耗时
    min_clearance_m   这一路 /scan 里出现过的最小距离（越小越危险）
    close_encounters  距离掉到 close_threshold_m 以下的次数（去抖，一次"贴脸"只算
                       一次，不是每帧都算）—— 没有接触传感器，这是"剐蹭"的代理指标，
                       不是真碰撞计数，报告里要说清楚这一点
    stuck_events      这一路 nav_watchdog 判定 STUCK 的次数（见 M1）

用法（需要先起好 gazebo + slam(localization) + task.launch.py，跟平时召唤一样）：
    ros2 run my_bot_task baseline_metrics.py --ros-args -p rooms:="['客厅','厨房']"

默认跑 current_map 拓扑里除 HOME 外的所有点各一次。结果打印成表格，同时写一份
JSON 到 ~/.maps/<map>/baseline_<timestamp>.json，方便以后改动导航参数（NeuPAN
调参等）前后各跑一次同一个脚本、直接 diff 两份 JSON 看有没有变好。
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

MAPS_ROOT = os.path.expanduser('~/.maps')


def resolve_map_name(explicit: str) -> str:
    """和 task_manager.py / slam.launch.py 一样的解析规则 —— 这是第三份，故意不共享。"""
    if explicit:
        return explicit
    try:
        with open(os.path.join(MAPS_ROOT, 'current_map')) as f:
            return f.read().strip()
    except OSError:
        return ''


class BaselineMetrics(Node):
    def __init__(self):
        super().__init__('baseline_metrics')

        self.declare_parameter('map_name', '')
        self.declare_parameter('rooms', [''])   # 空 => 用拓扑里除 HOME 外的全部点
        self.declare_parameter('close_threshold_m', 0.30)
        self.declare_parameter('trip_timeout_sec', 90.0)

        self.close_threshold = float(self.get_parameter('close_threshold_m').value)
        self.trip_timeout = float(self.get_parameter('trip_timeout_sec').value)

        self.points = self._load_topology()
        rooms_param = list(self.get_parameter('rooms').value or [])
        rooms_param = [r for r in rooms_param if r]
        if rooms_param:
            self.rooms = rooms_param
        else:
            self.rooms = [n for n in self.points if n.upper() != 'HOME']
        if not self.rooms:
            raise SystemExit('拓扑里没有可用的房间点 —— 先在 app 上标点')

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, '/goal_pose/cancel', 10)
        self.create_subscription(String, '/task/status', self._on_task_status, latched)
        self.create_subscription(String, '/nav_watchdog/status', self._on_watchdog, latched)
        # ⚠️ 用 /scan_filtered，不是 /scan —— 原始 /scan 有 <0.1m 的雷达盲区噪点
        # （laser_filters_sim.yaml 的 range_filter 就是干这个的，SLAM/Nav2 也吃
        # 过滤后的这份）。第一次跑漏了这条，六次召唤的 min_clearance 全部卡在
        # 0.100，明显是盲区钳位假象，不是真的贴到障碍物。
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan, 10)

        self.task_state = ''
        self.task_name = ''
        self.min_range_now = math.inf
        self.results = []

        self.get_logger().info(f'基线体检就绪：{len(self.rooms)} 个点 —— {", ".join(self.rooms)}')

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

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.task_state = data.get('state', '')
        current = data.get('current') or {}
        self.task_name = current.get('name', '')

    def _on_watchdog(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('status') == 'STUCK':
            self._stuck_hit = True

    def _on_scan(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self.min_range_now = min(self.min_range_now, min(valid))

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def _send_goal(self, point):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point['x'])
        msg.pose.position.y = float(point['y'])
        theta = float(point.get('theta', 0.0))
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.goal_pub.publish(msg)

    def run(self):
        for name in self.rooms:
            point = self.points.get(name)
            if point is None:
                self.get_logger().warn(f'拓扑里没有「{name}」，跳过')
                continue

            self.min_range_now = math.inf
            self._stuck_hit = False
            close_encounters = 0
            was_close = False
            t0 = time.monotonic()

            self._send_goal(point)
            self.get_logger().info(f'→ 召唤「{name}」')

            entered_nav = [False]   # 列表装着，方便闭包里改（Python 2/3 都行的老写法）

            def arrived():
                nonlocal close_encounters, was_close
                # 每次 spin_once 之后顺手更新一次"贴脸"去抖计数
                is_close = self.min_range_now < self.close_threshold
                if is_close and not was_close:
                    close_encounters += 1
                was_close = is_close

                # 先确认 task_manager 真的开始导航去我们要的这个点了，再谈"结束"——
                # 不然发完 goal 那一瞬间 task_state 还是上一轮的残留值，会被误判成
                # "已经不在 NAVIGATING 了"，第一轮循环直接假成功退出。
                if self.task_state == 'NAVIGATING' and self.task_name == name:
                    entered_nav[0] = True
                    return False
                if not entered_nav[0]:
                    return False
                if self.task_state == 'WAITING' and self.task_name == name:
                    return True       # 到点了
                return self.task_state != 'NAVIGATING'   # 中途被取消/失败/跳过

            ok = self._spin_until(arrived, self.trip_timeout)
            trip_sec = time.monotonic() - t0

            result = {
                'room': name,
                'ok': bool(ok and self.task_state == 'WAITING'),
                'trip_sec': round(trip_sec, 1),
                'min_clearance_m': None if math.isinf(self.min_range_now)
                                   else round(self.min_range_now, 3),
                'close_encounters': close_encounters,
                'stuck_events': int(self._stuck_hit),
            }
            self.results.append(result)
            self.get_logger().info(f'  {result}')

            if self.task_state == 'WAITING':
                self.cancel_pub.publish(Empty())   # 「我倒完了」，跳过 dwell 加速跑下一个
            time.sleep(1.0)

        self._report()

    def _report(self):
        print('\n==================== 基线体检结果 ====================')
        print(f'{"房间":<8}{"结果":<6}{"耗时(s)":<10}{"最小间距(m)":<14}{"贴脸次数":<10}{"卡死次数":<8}')
        for r in self.results:
            print(f'{r["room"]:<8}{"OK" if r["ok"] else "FAIL":<6}{r["trip_sec"]:<10}'
                  f'{r["min_clearance_m"]:<14}{r["close_encounters"]:<10}{r["stuck_events"]:<8}')
        n_ok = sum(1 for r in self.results if r['ok'])
        total_close = sum(r['close_encounters'] for r in self.results)
        total_stuck = sum(r['stuck_events'] for r in self.results)
        print(f'\n成功 {n_ok}/{len(self.results)}，贴脸总数 {total_close}，卡死总数 {total_stuck}')
        print(f'贴脸阈值 close_threshold_m={self.close_threshold} —— 这是代理指标，'
              f'不是真碰撞传感器读数（机器人没装接触/碰撞传感器）')

        name = resolve_map_name(str(self.get_parameter('map_name').value))
        if name:
            out_dir = os.path.join(MAPS_ROOT, name)
            out_path = os.path.join(out_dir, f'baseline_{int(time.time())}.json')
            try:
                with open(out_path, 'w') as f:
                    json.dump({
                        'close_threshold_m': self.close_threshold,
                        'results': self.results,
                    }, f, ensure_ascii=False, indent=2)
                print(f'已写入 {out_path}')
            except OSError as exc:
                print(f'写文件失败（不影响上面的结果）: {exc}')


def main():
    rclpy.init()
    node = BaselineMetrics()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
