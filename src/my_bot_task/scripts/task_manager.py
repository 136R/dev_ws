#!/usr/bin/env python3
"""移动垃圾桶的任务层：召唤 → 上门 → 等待 → 归位。

Nav2 的 NavigateToPose 是无状态的 —— 机器人跑到目标点之后什么也不会发生。
本节点在它上面套一层任务生命周期：

    [IDLE] ──队列非空──> [NAVIGATING] ──成功──> [WAITING] ──dwell 到 / 用户点完成──┐
       ▲                     │ 失败重试耗尽                                        │
       │                     └──────────────> _advance()  <───────────────────────┘
       │                                          │
       └──── 成功 ──── [RETURNING] <──队列空──────┘
                           │ 取消 / 重试耗尽
                           ▼
                       [STOPPED]  ──新任务进队──> [NAVIGATING]

【为什么订阅 /goal_pose】
app（ROS_Flutter_Gui_App）点拓扑点时，后端会发一个 PoseStamped 到 /goal_pose。
task.launch.py 把 bt_navigator 的这个订阅 remap 到 /nav2/goal_pose，于是本节点
独占 /goal_pose —— app 前端一行都不用改就能"召唤"。RViz 的 2D Goal Pose 同理。

【为什么 /goal_pose/cancel 就是"我倒完了"】
app 的「停止导航」按钮发 std_msgs/Empty 到 /goal_pose/cancel。
    WAITING 时收到 cancel = 用户手动点了"完成" → 立刻归位，不用等满 dwell。

【为什么要自己发 /task/nav_status】
但那个按钮【默认按不到】—— main_page.dart:1021 的可见性条件是
    navStatus == executing || navStatus == accepted
而机器人一到达，那次 NavigateToPose 就 succeeded 了，按钮当场消失。

app 订的是 gui_app_settings.json 的 NavToPoseStatusTopic（可配置）。所以本节点
自己发一个 GoalStatusArray 到 /task/nav_status，按【任务】的生命周期而不是【单次
导航】的生命周期来报：只要任务没结束（含 WAITING），就报 EXECUTING。
把 NavToPoseStatusTopic 指到这里，按钮的含义就正好变成"结束此刻正在做的这件事"
—— 跟 _on_cancel 的语义严丝合缝，而 Flutter 一行都不用改。
"""

import json
import math
import os
from collections import deque

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String

MAPS_ROOT = os.path.expanduser('~/.maps')

IDLE = 'IDLE'
NAVIGATING = 'NAVIGATING'
WAITING = 'WAITING'
RETURNING = 'RETURNING'
STOPPED = 'STOPPED'

# 任务状态 → 报给 app 的 GoalStatus。EXECUTING 让「停止导航」按钮可见。
# 注意 WAITING 也报 EXECUTING —— 那正是"我倒完了，回去吧"这个按钮存在的意义。
TASK_TO_GOAL_STATUS = {
    NAVIGATING: GoalStatus.STATUS_EXECUTING,
    WAITING: GoalStatus.STATUS_EXECUTING,
    RETURNING: GoalStatus.STATUS_EXECUTING,
    IDLE: GoalStatus.STATUS_SUCCEEDED,      # 按钮隐藏
    STOPPED: GoalStatus.STATUS_ABORTED,     # 按钮隐藏
}


def resolve_map_name(explicit: str) -> str:
    """map 参数优先，否则读 ~/.maps/current_map。

    slam.launch.py 和 keepout.launch.py 各有一份同样的实现 —— 那两个包互不依赖，
    这段解析刻意各留一份。这是第三份，同理。
    """
    if explicit:
        return explicit
    try:
        with open(os.path.join(MAPS_ROOT, 'current_map')) as f:
            return f.read().strip()
    except OSError:
        return ''


def yaw_from_quat(q) -> float:
    """平面 yaw。机器人是 2D 的，只取绕 z 的分量。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.declare_parameter('map_name', '')
        self.declare_parameter('match_radius', 0.5)
        self.declare_parameter('home_point_name', 'HOME')
        # 只声明类型、不给默认值：空列表 [] 在 rclpy 里推断成 NOT_SET，会抛
        # ParameterUninitializedException（加类型描述符也救不了）。没配就是没配。
        self.declare_parameter('home_pose', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dwell_sec', 120.0)
        self.declare_parameter('nav_retry_max', 2)
        self.declare_parameter('retry_backoff_sec', 2.0)
        self.declare_parameter('return_home', True)
        self.declare_parameter('poll_period_sec', 2.0)
        self.declare_parameter('settle_sec', 0.5)

        self.match_radius = float(self.get_parameter('match_radius').value)
        self.home_point_name = str(self.get_parameter('home_point_name').value)
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
        self.nav_retry_max = int(self.get_parameter('nav_retry_max').value)
        self.retry_backoff = float(self.get_parameter('retry_backoff_sec').value)
        self.return_home = bool(self.get_parameter('return_home').value)

        try:
            home_pose = list(self.get_parameter('home_pose').value or [])
        except rclpy.exceptions.ParameterUninitializedException:
            home_pose = []
        self.home_from_param = None
        if len(home_pose) == 3:
            self.home_from_param = {
                'name': self.home_point_name,
                'x': float(home_pose[0]),
                'y': float(home_pose[1]),
                'theta': float(home_pose[2]),
            }
        elif home_pose:
            self.get_logger().error(
                f'home_pose 要么留空、要么是 [x, y, theta] 三个数，收到 {home_pose} —— 已忽略')

        # ---- 状态 ----
        self.state = IDLE
        self.queue = deque()          # 待办任务，每项是 {name, x, y, theta}
        self.current = None           # 正在执行的任务（RETURNING 时是 home）
        self.points = {}              # 房间名 -> {name, x, y, theta}
        self.last_error = ''
        self.retries = 0

        self.goal_handle = None
        self.active_kind = None       # 'task' | 'home'
        self.dwell_timer = None
        self.dwell_deadline = None
        self.retry_timer = None
        self.tf_home = None           # TF 兜底抓到的待命点

        # 拓扑热重载（app 上加完点保存，不用重启本节点）
        self.topology_path = ''
        self.seen_mtime = 0.0
        self.pending_mtime = None

        # ---- ROS 接口 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # latched：后接的 `ros2 topic echo /task/status` 立刻就能拿到当前状态
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/task/status', latched)

        # 给 app 的「停止导航」按钮用。把 gui_app_settings.json 的
        # NavToPoseStatusTopic 指到这里，按钮才会在 WAITING 时显示（见模块开头说明）。
        self.nav_status_pub = self.create_publisher(
            GoalStatusArray, '/task/nav_status', latched)

        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(Empty, '/goal_pose/cancel', self._on_cancel, 10)
        self.create_subscription(Empty, '/task/cancel_all', self._on_cancel_all, 10)

        poll = float(self.get_parameter('poll_period_sec').value)
        self.settle = float(self.get_parameter('settle_sec').value)
        self.create_timer(poll, self._poll_topology)
        self.create_timer(1.0, self._publish_status)   # 让 dwell_remaining 能倒数

        self._reload_topology()
        self._publish_status()
        self.get_logger().info(
            f'任务层就绪：dwell={self.dwell_sec}s, return_home={self.return_home}')

    # ================= 拓扑点 =================

    def _topology_file(self) -> str:
        name = resolve_map_name(str(self.get_parameter('map_name').value))
        if not name:
            return ''
        return os.path.join(MAPS_ROOT, name, f'{name}.topology')

    def _mtime(self, path: str) -> float:
        try:
            return os.path.getmtime(path)
        except OSError:
            return 0.0

    def _poll_topology(self):
        path = self._topology_file()
        if path != self.topology_path:      # 用户在 app 上换了地图
            self.topology_path = path
            self.seen_mtime = 0.0
        now = self._mtime(path)
        if now == 0.0 or now == self.seen_mtime:
            return
        # 文件还在写 → 等它静下来，别读到写了一半的 JSON
        if self.pending_mtime != now:
            self.pending_mtime = now
            return
        self.seen_mtime, self.pending_mtime = now, None
        self._reload_topology()

    def _reload_topology(self):
        """读 ~/.maps/<current_map>/<name>.topology。

        文件不存在 / points 为空都【不算错误】—— 用户很可能先起了节点再去 app 上标点。
        只 warn 并继续轮询，绝不 raise 把节点搞崩。
        """
        path = self._topology_file()
        self.topology_path = path
        if not path:
            self.get_logger().warn('还没有 current_map，等 app 选图', throttle_duration_sec=30.0)
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except OSError:
            self.get_logger().warn(
                f'读不到 {path} —— 去 app 上标几个导航点并保存', throttle_duration_sec=30.0)
            return
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'{path} 不是合法 JSON: {exc}')
            return

        points = {}
        dupes = []
        for p in data.get('points', []):
            name = str(p.get('name', '')).strip()
            if not name:
                continue
            if name in points:
                # 名字是任务的唯一标识（去重、队列、日志全靠它），重名会互相覆盖。
                # 名字本身随便起（人名、中文都行），但必须唯一。
                dupes.append(name)
            # 刻意忽略 type 字段：app 的 Dart 侧写 'ChargeStation'、C++ 后端认
            # 'ChargingStation'，两边对不上（已知的上游 bug）。待命点靠名字约定。
            points[name] = {
                'name': name,
                'x': float(p.get('x', 0.0)),
                'y': float(p.get('y', 0.0)),
                'theta': float(p.get('theta', 0.0)),
            }
        if dupes:
            self.get_logger().error(
                f'⚠️ 拓扑里有重名的点: {", ".join(sorted(set(dupes)))} —— '
                f'同名点会互相覆盖，只有【最后一个】生效！去 app 上改名。')
        self.points = points
        self.seen_mtime = self._mtime(path)
        self.get_logger().info(f'拓扑已加载（{len(points)} 个点）: {", ".join(points) or "空"}')

    def _home(self):
        """待命点。home_pose 参数 > 拓扑里的 HOME 点 > 启动时的 TF 位姿。"""
        if self.home_from_param:
            return self.home_from_param
        for name, p in self.points.items():
            if name.upper() == self.home_point_name.upper():
                return p
        if self.tf_home is None:
            self.tf_home = self._current_pose()
            if self.tf_home:
                self.get_logger().warn(
                    f'拓扑里没有名叫 {self.home_point_name} 的点，也没设 home_pose 参数 —— '
                    f'退而把当前位置 ({self.tf_home["x"]:.2f}, {self.tf_home["y"]:.2f}) 当待命点。'
                    f'去 app 上加一个名叫 {self.home_point_name} 的导航点。')
        return self.tf_home

    def _current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        return {'name': self.home_point_name, 'x': t.x, 'y': t.y,
                'theta': yaw_from_quat(tf.transform.rotation)}

    # ================= 召唤入口 =================

    def _on_goal(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        theta = yaw_from_quat(msg.pose.orientation)

        # 就近匹配到房间名。匹配不到（RViz 随手点的空地）不拒绝，退化成匿名任务。
        task = None
        best = self.match_radius
        for p in self.points.values():
            d = math.hypot(p['x'] - x, p['y'] - y)
            if d <= best:
                best, task = d, dict(p)
        if task is None:
            task = {'name': f'点({x:.2f}, {y:.2f})', 'x': x, 'y': y, 'theta': theta}

        # 待命点不是任务 —— 它是"回家"，不该被当成召唤排进队列
        if task['name'].upper() == self.home_point_name.upper():
            self.get_logger().info(f'{task["name"]} 是待命点，不当作召唤')
            return

        name = task['name']
        if any(t['name'] == name for t in self.queue) or (
                self.current and self.active_kind == 'task'
                and self.current['name'] == name):
            self.get_logger().info(f'「{name}」已在队列/正在服务 —— 忽略重复召唤')
            return

        self.queue.append(task)
        self.get_logger().info(f'召唤「{name}」入队（队长 {len(self.queue)}）')

        if self.state in (IDLE, STOPPED):
            self._advance()
        elif self.state == RETURNING:
            # 正在回家的路上被叫住 —— 取消归位，直接去新目标（不先回家）
            self.get_logger().info('归位途中被召唤 → 取消归位，直奔新目标')
            self._cancel_active()
        else:
            self._publish_status()

    def _on_cancel(self, _msg: Empty):
        """app 的「停止导航」按钮。cancel = 结束此刻正在做的这件事，然后往下走。"""
        if self.state == WAITING:
            self.get_logger().info('用户点了完成 → 提前结束等待')
            self._clear_dwell()
            self._advance()
        elif self.state in (NAVIGATING, RETURNING):
            self.get_logger().info(f'取消当前{"归位" if self.state == RETURNING else "导航"}')
            self._cancel_active()
        else:
            self.get_logger().info(f'{self.state} 状态下收到 cancel —— 无事可做')

    def _on_cancel_all(self, _msg: Empty):
        n = len(self.queue)
        self.queue.clear()
        self.get_logger().info(f'清空队列（丢弃 {n} 个待办）')
        if self.state in (NAVIGATING, WAITING, RETURNING):
            self._on_cancel(Empty())
        else:
            self._publish_status()

    # ================= 状态机 =================

    def _advance(self):
        """一件事做完了 —— 决定下一步。"""
        self._clear_dwell()
        self.retries = 0

        if self.queue:
            task = self.queue.popleft()
            self._goto(task, 'task')
            return

        if not self.return_home:
            self._enter(IDLE)
            return

        home = self._home()
        if home is None:
            self.get_logger().warn('没有待命点可回（TF 也拿不到）—— 原地待命')
            self._enter(IDLE)
            return
        if self._at(home):
            self._enter(IDLE)
            return
        self._goto(home, 'home')

    def _at(self, p, tol=0.3) -> bool:
        pose = self._current_pose()
        if pose is None:
            return False
        return math.hypot(pose['x'] - p['x'], pose['y'] - p['y']) <= tol

    def _goto(self, task, kind):
        self.current = task
        self.active_kind = kind
        self._enter(RETURNING if kind == 'home' else NAVIGATING)
        self._send_nav_goal(task)

    def _enter(self, state):
        if state in (IDLE, STOPPED):
            self.current = None
            self.active_kind = None
            self.goal_handle = None
        self.state = state
        self._publish_status()

    # ================= NavigateToPose =================

    def _send_nav_goal(self, task):
        # 用 server_is_ready() 而不是 wait_for_server(timeout) —— 后者会在单线程
        # executor 的回调里阻塞 spin。Nav2 起得比本节点慢是正常的，等就是了，
        # 而且这种等待【不消耗 nav_retry_max】（那是留给真正导航失败的）。
        if not self.nav.server_is_ready():
            self.get_logger().warn(
                'navigate_to_pose 还没就绪，1s 后重试', throttle_duration_sec=10.0)
            self._clear_retry()
            self.retry_timer = self.create_timer(1.0, lambda: self._do_retry(task))
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = task['x']
        goal.pose.pose.position.y = task['y']
        goal.pose.pose.orientation.z = math.sin(task['theta'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(task['theta'] / 2.0)

        self.get_logger().info(
            f'{"归位 →" if self.active_kind == "home" else "前往"}「{task["name"]}」'
            f'({task["x"]:.2f}, {task["y"]:.2f})')
        self.nav.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - action 通信失败不该拖垮节点
            self._fail(f'发目标失败: {exc}')
            return
        if not handle.accepted:
            self._fail('Nav2 拒绝了目标')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self._fail(f'取导航结果失败: {exc}')
            return

        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.retries = 0
            if self.active_kind == 'home':
                self.get_logger().info('已归位，待命中')
                self._enter(IDLE)
            else:
                self.get_logger().info(
                    f'到达「{self.current["name"]}」—— 等 {self.dwell_sec:.0f}s '
                    f'（app 上点「停止导航」可提前结束）')
                self._start_dwell()
            return

        if status == GoalStatus.STATUS_CANCELED:
            # 两种来源：用户按了停止 / 归位途中被新召唤打断。都是"这件事结束了，往下走"
            if self.active_kind == 'home' and not self.queue:
                self.get_logger().info('归位被取消 —— 原地停住')
                self._enter(STOPPED)
            else:
                self._advance()
            return

        self._fail(f'导航失败 (GoalStatus={status})')

    def _fail(self, reason):
        self.last_error = reason
        self.retries += 1
        if self.retries <= self.nav_retry_max:
            self.get_logger().warn(
                f'{reason} —— {self.retry_backoff:.0f}s 后重试 '
                f'({self.retries}/{self.nav_retry_max})')
            task = self.current
            self._clear_retry()
            self.retry_timer = self.create_timer(
                self.retry_backoff, lambda: self._do_retry(task))
            return

        self.get_logger().error(f'{reason} —— 重试耗尽，放弃「{self.current["name"]}」')
        if self.active_kind == 'home':
            # 回不了家就别原地空转了，停住等人
            self._enter(STOPPED)
        else:
            self._advance()

    def _do_retry(self, task):
        self._clear_retry()
        self._send_nav_goal(task)

    def _clear_retry(self):
        if self.retry_timer is not None:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

    def _cancel_active(self):
        if self.goal_handle is None:
            self._advance()
            return
        self.goal_handle.cancel_goal_async()
        # 不在这里改状态 —— 等 _on_result 收到 STATUS_CANCELED 再走 _advance()

    # ================= 等待 =================

    def _start_dwell(self):
        self._clear_dwell()
        self.dwell_deadline = self.get_clock().now().nanoseconds * 1e-9 + self.dwell_sec
        # Humble 的 rclpy 没有一次性 timer，回调里自己销毁
        self.dwell_timer = self.create_timer(self.dwell_sec, self._on_dwell_done)
        self._enter(WAITING)

    def _on_dwell_done(self):
        self.get_logger().info('等待结束 → 归位')
        self._clear_dwell()
        self._advance()

    def _clear_dwell(self):
        if self.dwell_timer is not None:
            self.destroy_timer(self.dwell_timer)
            self.dwell_timer = None
        self.dwell_deadline = None

    # ================= 状态播报 =================

    def _publish_status(self):
        remaining = 0.0
        if self.state == WAITING and self.dwell_deadline is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            remaining = max(0.0, self.dwell_deadline - now)

        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'current': self.current,
            'queue': [t['name'] for t in self.queue],
            'queue_len': len(self.queue),
            'dwell_remaining': round(remaining, 1),
            'last_error': self.last_error,
            'stamp': round(self.get_clock().now().nanoseconds * 1e-9, 3),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

        arr = GoalStatusArray()
        gs = GoalStatus()
        gs.status = TASK_TO_GOAL_STATUS[self.state]
        gs.goal_info.stamp = self.get_clock().now().to_msg()
        arr.status_list = [gs]           # app 取 status_list.last
        self.nav_status_pub.publish(arr)


def main():
    rclpy.init()
    node = TaskManager()
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
