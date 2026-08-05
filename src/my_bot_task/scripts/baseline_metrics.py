#!/usr/bin/env python3
"""动态环境导航基线体检（M2）。

不改任何导航代码，按拓扑点名字挨个跑一遍，边跑边记指标。调参前后各跑一次、
diff 两份 JSON，就能回答"这次改动到底有没有变好"。

指标
----
    trip_sec          单趟耗时
    min_clearance_m   这一路 /scan_filtered 里出现过的最小距离（越小越危险）
    close_encounters  距离掉到 close_threshold_m 以下的次数（去抖，一次"贴脸"只算
                       一次，不是每帧都算）—— 没有接触传感器，这是"剐蹭"的代理指标，
                       不是真碰撞计数，报告里要说清楚这一点
    stuck_events      nav_watchdog 判定 STUCK 的次数（仅 task 模式，direct 模式记 None）
    recovery_count    Nav2 自己进恢复链的次数。比 stuck_events 灵敏得多 —— watchdog 要
                       30s 位移不足才判一次卡死，而 Nav2 的恢复链几秒就能触发一轮，
                       "频繁僵死"这个症状主要靠这个数看出来
    path_ratio        实走里程 ÷ 起终点直线距离。量化"绕远 / 来回抖"，1.0 = 走直线
    stall_sec         cmd_vel 近零速的累计时长。量化"停着不动"，是让行策略的直接效果指标

两种模式
--------
    task    起了 task.launch.py：发 /goal_pose 给任务层，从 /task/status 的
            NAVIGATING→WAITING 判定到达。指标最全（含 stuck_events）。
    direct  只起了 nav.launch.py：任务层不在，/goal_pose 会被 bt_navigator 直接接管
            （没有 remap 到 /nav2/goal_pose），且没有 /task/status 可判到达 —— 改用
            NavigateToPose action 的返回状态判定。stuck_events 不可用（nav_watchdog
            也在任务层里），由 recovery_count 顶替。

    默认 mode:=auto —— 探测 /task/status 有没有 publisher 来自动选。

用法
----
    # A. 起了任务层（gazebo + slam + task.launch.py），跟平时召唤一样
    ros2 run my_bot_task baseline_metrics.py --ros-args \
        -p use_sim_time:=true -p rooms:="['客厅','厨房']" -p repeat:=3

    # B. 只起了 nav.launch.py
    ros2 launch my_bot_nav nav.launch.py use_sim_time:=true \
        params_file:=~/dev_ws/src/my_bot_nav/config/sim/nav2_params_neupan_smac2d.yaml
    ros2 run my_bot_task baseline_metrics.py --ros-args -p use_sim_time:=true -p repeat:=3

⚠️ 仿真下务必传 use_sim_time:=true，否则 goal 的 header.stamp 打的是墙钟，和 Nav2
   的 /clock 对不上。（超时计时故意仍用墙钟 —— 万一 /clock 没在发，用 ROS 时钟会
   永远等不到超时而假死。所以 RTF 明显偏离 1 时 trip_sec/stall_sec 会有偏差。）

默认跑 current_map 拓扑里除 HOME 外的所有点各一次。结果打印成表格，同时写一份
JSON 到 ~/.maps/<map>/baseline_<timestamp>.json。
"""

import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

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
        self.declare_parameter('repeat', 1)
        self.declare_parameter('mode', 'auto')          # auto | task | direct
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        # BT 里恢复链的 RoundRobin 节点名（navigate_to_pose_custom.xml 的
        # <RoundRobin name="RecoveryActions">）。换了 BT 或改了名字要同步改这里，
        # 否则 recovery_count 会一直是 0 —— 那不是"没进恢复"，是没数到。
        self.declare_parameter('recovery_node_name', 'RecoveryActions')
        self.declare_parameter('stall_speed_eps', 0.02)  # 线/角速度都低于它算"没在动"

        self.close_threshold = float(self.get_parameter('close_threshold_m').value)
        self.trip_timeout = float(self.get_parameter('trip_timeout_sec').value)
        self.repeat = max(1, int(self.get_parameter('repeat').value))
        self.recovery_node = str(self.get_parameter('recovery_node_name').value)
        self.stall_eps = float(self.get_parameter('stall_speed_eps').value)

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
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log',
                                 self._on_bt_log, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Twist, str(self.get_parameter('cmd_vel_topic').value),
                                 self._on_cmd_vel, 10)

        # path_ratio 的分子走 odom 累积（连续、无跳变），分母的起点走 TF 查 map 系
        # —— 目标点是 map 系的，两头必须同系才能算直线距离。故意不用 map 系累积里程：
        # SLAM 修正会让 map 位姿瞬间跳变，那种跳变加进里程里就是凭空多出来的路程。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.task_state = ''
        self.task_name = ''
        # 必须分开：scan_min_now 是【这一帧】的值（贴脸的进入/离开判定要用它，
        # 共用累积值会让判据单调化、计数恒为 1），min_range_now 是整趟最小值只用于报告。
        self.scan_min_now = math.inf
        self.min_range_now = math.inf
        self.results = []

        self._last_wd_status = ''
        self._stuck_count = 0
        self._recovery_count = 0
        self._odom_xy = None
        self._odom_dist = 0.0
        self._stall_sec = 0.0
        self._last_cmd_wall = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._send_future = None
        self._result_future = None
        self._direct_ok = False
        self._feedback_count = 0
        self._trip_note = None

        self.mode = self._resolve_mode(str(self.get_parameter('mode').value))
        self.get_logger().info(
            f'基线体检就绪（{self.mode} 模式，{self.repeat} 轮）：'
            f'{len(self.rooms)} 个点 —— {", ".join(self.rooms)}')

    # ── 模式选择 ──────────────────────────────────────────────────────────
    def _resolve_mode(self, requested):
        if requested in ('task', 'direct'):
            return requested
        # auto：探测 /task/status 有没有 publisher。DDS 发现需要时间，光 count 一次
        # 会在节点刚起来时误判成 direct，所以 spin 着等几秒。
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.count_publishers('/task/status') > 0:
                return 'task'
        self.get_logger().info('没探测到 /task/status —— 按 direct 模式跑（stuck_events 不可用）')
        return 'direct'

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

    # ── 订阅回调 ──────────────────────────────────────────────────────────
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
        status = data.get('status', '')
        # 只在【状态变化】时计数。nav_watchdog 自己有 triggered 自锁，一次卡死只发一条
        # STUCK，本来直接计数也对；但这个话题是 TRANSIENT_LOCAL(latched)，订阅一建立
        # 就会重放最后一条，重放的若正好是 STUCK 就会凭空多一次。写法对齐 soak_test.py。
        if status == 'STUCK' and status != self._last_wd_status:
            self._stuck_count += 1
        self._last_wd_status = status

    def _on_bt_log(self, msg: BehaviorTreeLog):
        # behavior_tree_log 只在节点状态【变化】时发事件，所以这里天然是边沿，
        # 不需要再去抖。RoundRobin 转 RUNNING 一次 = 进了一轮恢复。
        for event in msg.event_log:
            if event.node_name == self.recovery_node and event.current_status == 'RUNNING':
                self._recovery_count += 1

    def _on_scan(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self.scan_min_now = min(valid)
            self.min_range_now = min(self.min_range_now, self.scan_min_now)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        if self._odom_xy is not None:
            self._odom_dist += math.hypot(p.x - self._odom_xy[0], p.y - self._odom_xy[1])
        self._odom_xy = (p.x, p.y)

    def _on_cmd_vel(self, msg: Twist):
        now = time.monotonic()
        if self._last_cmd_wall is not None:
            dt = now - self._last_cmd_wall
            # dt 上限兜底：话题断流再恢复时，两帧间隔可能是几十秒，那段时间机器人在
            # 干什么并不知道，不能一股脑算成"停着"。
            if dt < 1.0 and abs(msg.linear.x) < self.stall_eps \
                    and abs(msg.angular.z) < self.stall_eps:
                self._stall_sec += dt
        self._last_cmd_wall = now

    # ── 工具 ──────────────────────────────────────────────────────────────
    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def _map_xy(self):
        """当前 map 系位置。拿不到（TF 还没起来/没跑定位）返回 None，指标降级而不是崩。"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def _make_pose(self, point) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point['x'])
        msg.pose.position.y = float(point['y'])
        theta = float(point.get('theta', 0.0))
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        return msg

    # ── 两种模式各自的"发目标"和"到了吗" ──────────────────────────────────
    def _send_goal(self, point):
        if self.mode == 'task':
            self.goal_pub.publish(self._make_pose(point))
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(point)
        self._send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback)
        self._result_future = None
        self._direct_ok = False

    def _on_nav_feedback(self, _msg):
        # 内容不关心，只数条数 —— 用来证明"这次导航真的跑起来过"，见 _direct_finished。
        self._feedback_count += 1

    def _direct_finished(self) -> bool:
        """direct 模式：action 走完（成功或失败）返回 True。"""
        if self._result_future is None:
            if self._send_future is not None and self._send_future.done():
                handle = self._send_future.result()
                self._send_future = None
                if handle is None or not handle.accepted:
                    self.get_logger().warn('  目标被 bt_navigator 拒绝')
                    self._trip_note = 'rejected'
                    return True
                self._result_future = handle.get_result_async()
            return False
        if self._result_future.done():
            res = self._result_future.result()
            ok = (res.status == GoalStatus.STATUS_SUCCEEDED)
            if not ok:
                self.get_logger().warn(f'  导航失败，action status={res.status}')
            elif self._feedback_count == 0:
                # 竞态防御：goal 刚 accept 就可能拿到不属于这次导航的 result
                # （实测出现过 3m 目标 0.1s "成功"、里程显示没动）。一条 feedback
                # 都没有 = 没真跑过，判为无效趟（不是失败）。
                self.get_logger().warn('  一条 feedback 都没收到就返回成功 —— 判为无效趟')
                ok = False
                self._trip_note = 'no_feedback'
            self._direct_ok = ok
            return True
        return False

    # ── 主流程 ────────────────────────────────────────────────────────────
    def run(self):
        if self.mode == 'direct':
            self.get_logger().info('等待 navigate_to_pose action server…')
            if not self._nav_client.wait_for_server(timeout_sec=15.0):
                raise SystemExit('连不上 navigate_to_pose —— nav.launch.py 起了吗？')

        for round_idx in range(1, self.repeat + 1):
            if self.repeat > 1:
                self.get_logger().info(f'═══ 第 {round_idx}/{self.repeat} 轮 ═══')
            for name in self.rooms:
                point = self.points.get(name)
                if point is None:
                    self.get_logger().warn(f'拓扑里没有「{name}」，跳过')
                    continue
                self.results.append(self._run_one_trip(name, point, round_idx))

        self._report()

    def _run_one_trip(self, name, point, round_idx):
        # 每趟归零。odom 里程用"清零累加器"而不是记起点差值 —— 要的是实走路程，
        # 不是位移。
        self.min_range_now = math.inf
        self.scan_min_now = math.inf
        self._stuck_count = 0
        self._recovery_count = 0
        self._odom_dist = 0.0
        self._stall_sec = 0.0
        self._last_cmd_wall = None
        self._feedback_count = 0
        self._trip_note = None
        close_encounters = 0
        was_close = False

        # TF 可能还没就绪，先 spin 一小会儿再取起点
        self._spin_until(lambda: self._map_xy() is not None, 2.0)
        start_xy = self._map_xy()

        t0 = time.monotonic()
        self._send_goal(point)
        self.get_logger().info(f'→ 召唤「{name}」')

        entered_nav = [False]   # 列表装着，方便闭包里改（Python 2/3 都行的老写法）

        def arrived():
            nonlocal close_encounters, was_close
            # 贴脸去抖计数。必须用当前帧值，累积最小值单调递减会让边沿检测失效。
            is_close = self.scan_min_now < self.close_threshold
            if is_close and not was_close:
                close_encounters += 1
            was_close = is_close

            if self.mode == 'direct':
                return self._direct_finished()

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

        if self.mode == 'direct':
            ok = bool(ok and self._direct_ok)
        else:
            ok = bool(ok and self.task_state == 'WAITING')

        # path_ratio：实走里程 ÷ 起终点直线距离。直线距离过小（原地目标）时不算，
        # 否则分母趋零会放出一个没有意义的巨大比值。
        path_ratio = None
        if start_xy is not None:
            straight = math.hypot(float(point['x']) - start_xy[0],
                                  float(point['y']) - start_xy[1])
            if straight > 0.3:
                path_ratio = round(self._odom_dist / straight, 2)

        result = {
            'round': round_idx,
            'room': name,
            'ok': ok,
            # valid=False 表示"这趟没测成"，和"导航失败"是两回事：失败是被测系统的
            # 表现（要算进成功率），无效是测量本身出了岔（要整趟剔除）。
            'valid': self._trip_note is None,
            'note': self._trip_note,
            'trip_sec': round(trip_sec, 1),
            'min_clearance_m': None if math.isinf(self.min_range_now)
                               else round(self.min_range_now, 3),
            'close_encounters': close_encounters,
            # direct 模式下 nav_watchdog 不在，记 None 而不是 0 —— 0 会被误读成
            # "跑了一趟一次没卡"，而事实是根本没测。
            'stuck_events': self._stuck_count if self.mode == 'task' else None,
            'recovery_count': self._recovery_count,
            'path_ratio': path_ratio,
            'stall_sec': round(self._stall_sec, 1),
        }
        self.get_logger().info(f'  {result}')

        if self.mode == 'task' and self.task_state == 'WAITING':
            self.cancel_pub.publish(Empty())   # 「我倒完了」，跳过 dwell 加速跑下一个
        time.sleep(1.0)
        return result

    # ── 报告 ──────────────────────────────────────────────────────────────
    def _report(self):
        def fmt(v):
            return '—' if v is None else str(v)

        print('\n' + '=' * 92)
        print(f'基线体检结果（{self.mode} 模式，{self.repeat} 轮）')
        print('=' * 92)
        head = (f'{"轮":<4}{"房间":<10}{"结果":<7}{"耗时(s)":<10}{"最小间距":<11}'
                f'{"贴脸":<7}{"恢复":<7}{"卡死":<7}{"绕路比":<9}{"停滞(s)":<9}')
        print(head)
        print('-' * 92)
        for r in self.results:
            verdict = '无效' if not r['valid'] else ('OK' if r['ok'] else 'FAIL')
            print(f'{r["round"]:<4}{r["room"]:<10}{verdict:<7}'
                  f'{r["trip_sec"]:<10}{fmt(r["min_clearance_m"]):<11}'
                  f'{r["close_encounters"]:<7}{r["recovery_count"]:<7}'
                  f'{fmt(r["stuck_events"]):<7}{fmt(r["path_ratio"]):<9}'
                  f'{r["stall_sec"]:<9}')

        # 汇总只统计有效趟 —— 无效趟的 path_ratio≈0.01、trip_sec≈0.1 会把均值带偏
        valid = [r for r in self.results if r['valid']]
        n_invalid = len(self.results) - len(valid)
        n = len(valid)
        n_ok = sum(1 for r in valid if r['ok'])
        total_close = sum(r['close_encounters'] for r in valid)
        total_recovery = sum(r['recovery_count'] for r in valid)
        total_stuck = sum(r['stuck_events'] or 0 for r in valid)
        ratios = [r['path_ratio'] for r in valid if r['path_ratio'] is not None]
        trips = [r['trip_sec'] for r in valid]
        stalls = [r['stall_sec'] for r in valid]

        print('-' * 92)
        print(f'成功 {n_ok}/{n}　贴脸 {total_close}　进恢复链 {total_recovery}　'
              f'卡死 {fmt(total_stuck if self.mode == "task" else None)}')
        if n_invalid:
            notes = {r['note'] for r in self.results if not r['valid']}
            print(f'⚠️ {n_invalid} 趟无效（{", ".join(sorted(notes))}），已从上面的汇总中剔除')
        if ratios:
            print(f'绕路比  均值 {sum(ratios) / len(ratios):.2f}　最差 {max(ratios):.2f}')
        if trips:
            print(f'耗时(s) 均值 {sum(trips) / len(trips):.1f}　最差 {max(trips):.1f}')
        if stalls:
            print(f'停滞(s) 均值 {sum(stalls) / len(stalls):.1f}　最差 {max(stalls):.1f}')
        print(f'\n贴脸阈值 close_threshold_m={self.close_threshold} —— 这是代理指标，'
              f'不是真碰撞传感器读数（机器人没装接触/碰撞传感器）')
        if self.mode == 'direct':
            print('direct 模式：nav_watchdog 不在，卡死一列为「—」（没测，不是 0），'
                  '看「进恢复链」那列')

        name = resolve_map_name(str(self.get_parameter('map_name').value))
        if name:
            out_dir = os.path.join(MAPS_ROOT, name)
            out_path = os.path.join(out_dir, f'baseline_{int(time.time())}.json')
            try:
                with open(out_path, 'w') as f:
                    json.dump({
                        'mode': self.mode,
                        'repeat': self.repeat,
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
