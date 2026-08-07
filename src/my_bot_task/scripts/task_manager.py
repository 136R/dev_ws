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

【为什么要自开一个 HTTP 端口】
app 后端是【固定 schema 的 protobuf 桥】，消费不了 /task/status 这种自定义类型 ——
加一个类型要同时改 C++ 后端 + .proto + Dart 生成代码，比改前端还贵。

所以本节点自己开一个 HTTP 口（默认 :8090），前端 1Hz 轮询：
    GET  /task/status  → 当前状态 JSON（state / current / queue / dwell_remaining）
    POST /task/skip    → "我倒完了" = 跳过等待（等价于发 /goal_pose/cancel）
    POST /task/cancel_all → 清空队列
    POST /task/force_resume → 人工解除低电锁定（override_sec 内不再判低电）

前端拿到 state 才知道现在是不是 WAITING，才能【只在等待时】显示那个
「我倒完了，回去吧」按钮 —— 语义唯一，不跟「停止导航」混。

【低电为什么复用 STOPPED，而不是新增 LOW_BATTERY 状态】

前端 `task_status.dart` 只定义了 isIdle/isNavigating/isWaiting/isReturning/isStopped
五个 getter，且 `shouldShowCard => !isIdle`；`task_card.dart` 又只在 isStopped 时
显示 last_error。新增一个状态的后果是：卡片会弹出来（因为 != IDLE），但五个 getter
全为 false → 卡片内容无分支可渲染，**而且低电拒绝召唤时用户看不到任何提示**，
表现为"点了没反应"。要修就得改 Dart 并重新 build + 部署 Flutter web。

复用 STOPPED 后前端一行都不用改，last_error 直接显示"电量低（xx%），已回到待命点"。
状态区分交给 JSON 里的 battery_low 字段（前端暂不消费，排障用）。
STOPPED 原有语义是"新召唤会拉回 NAVIGATING"，而低电要拒绝 —— 这个行为差异由独立的
self.low_battery 标志控制，与 state 字段无关，两者不冲突。
"""

import json
import math
import os
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String

MAPS_ROOT = os.path.expanduser('~/.maps')

IDLE = 'IDLE'
NAVIGATING = 'NAVIGATING'
WAITING = 'WAITING'
RETURNING = 'RETURNING'
STOPPED = 'STOPPED'


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


class _TaskHttpHandler(BaseHTTPRequestHandler):
    """给 app 前端用的极简 HTTP 接口。

    ⚠️ 这些回调跑在【HTTP 线程】，不是 rclpy 的 executor 线程。
    所以绝不在这里直接调节点的方法（那会和 spin 里的回调抢状态机）——
    只往 node.pending_cmds 里塞一个字符串，由节点自己的 timer 排空。
    """

    server_version = 'my_bot_task'

    def _send(self, code, body=b'', ctype='application/json'):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        # Flutter Web 是从后端的 :8080 加载的，打本节点的 :8090 属于跨域
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_OPTIONS(self):          # noqa: N802 - BaseHTTPRequestHandler 的命名约定
        self._send(204)

    def do_GET(self):              # noqa: N802
        if self.path.split('?')[0] == '/task/status':
            self._send(200, self.server.node.status_json.encode('utf-8'))
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):             # noqa: N802
        path = self.path.split('?')[0]
        if path in ('/task/skip', '/task/cancel_all', '/task/force_resume'):
            self.server.node.pending_cmds.append(path)
            self._send(200, b'{"ok":true}')
        else:
            self._send(404, b'{"error":"not found"}')

    def log_message(self, *_args):
        pass                       # 别把每个轮询请求都打到 ROS 日志里


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
        self.declare_parameter('http_port', 8090)
        # ETA = 剩余路径长度 ÷ 这个速度。不用 Nav2 自己的 estimated_time_remaining ——
        # 它是"剩余距离 ÷ 当前瞬时线速度"，机器人一转弯/过门减速就炸到几百秒再跳回来。
        # 默认 0.2 < NeuPAN 的 ref_speed 0.3，因为过门和转弯会拉低实际均速。按实测调。
        self.declare_parameter('eta_speed_mps', 0.2)

        # 低电归位。⚠️ 这七个刻意【不缓存】—— _check_battery() 每次都重读参数服务器，
        # 这样 `ros2 param set /task_manager low_voltage 10.9` 能当场生效。
        # 阈值是要在实机上反复试的东西，每次都重启节点会把标定过程拖垮。
        self.declare_parameter('low_voltage', 10.70)
        self.declare_parameter('resume_voltage', 11.19)
        self.declare_parameter('critical_voltage', 9.90)
        self.declare_parameter('low_debounce_sec', 30.0)
        self.declare_parameter('resume_debounce_sec', 120.0)
        self.declare_parameter('battery_stale_sec', 30.0)
        self.declare_parameter('override_sec', 600.0)

        self.match_radius = float(self.get_parameter('match_radius').value)
        self.home_point_name = str(self.get_parameter('home_point_name').value)
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
        self.nav_retry_max = int(self.get_parameter('nav_retry_max').value)
        self.retry_backoff = float(self.get_parameter('retry_backoff_sec').value)
        self.return_home = bool(self.get_parameter('return_home').value)
        self.eta_speed = max(0.01, float(self.get_parameter('eta_speed_mps').value))

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

        # 来自 Nav2 feedback（只信 distance_remaining，ETA 自己算，见 _on_feedback）
        self.distance_remaining = 0.0
        self.eta_sec = 0.0

        # ---- 电量 ----
        self.batt_v = None            # battery_monitor 滤波后的电压（决策用的就是它）
        self.batt_pct = None          # 百分比，只用来写进 last_error 给人看
        self.batt_stamp = None        # 最后一条 /battery_status 的到达时刻
        self.batt_present = False
        self.battery_stale = True     # 没数据 = 失联，而失联一律【失效开放】
        self.low_battery = False      # 低电锁存：拒绝新召唤 + 到家后停 STOPPED
        self.battery_critical = False # 物理地板：就地停，连归位都不再尝试
        self.low_since = None         # 连续低于 low_voltage 的起点（去抖用）
        self.resume_since = None      # 连续高于 resume_voltage 的起点
        self.override_until = None    # 人工覆盖到期时刻

        # HTTP 线程 ↔ ROS 线程之间只靠这两个：一个只读的快照，一个只写的命令队列。
        # deque 的 append/popleft 在 CPython 里是原子的，够用了，不用上锁。
        self.status_json = '{}'
        self.pending_cmds = deque()

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

        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(Empty, '/goal_pose/cancel', self._on_cancel, 10)
        self.create_subscription(Empty, '/task/cancel_all', self._on_cancel_all, 10)
        self.create_subscription(BatteryState, '/battery_status', self._on_battery, 10)

        poll = float(self.get_parameter('poll_period_sec').value)
        self.settle = float(self.get_parameter('settle_sec').value)
        self.create_timer(poll, self._poll_topology)
        self.create_timer(1.0, self._publish_status)   # 让 dwell_remaining 能倒数
        self.create_timer(1.0, self._check_battery)
        self.create_timer(0.2, self._drain_http_cmds)  # 排空 HTTP 线程塞进来的命令

        self._reload_topology()
        self._publish_status()
        self._start_http()
        self.get_logger().info(
            f'任务层就绪：dwell={self.dwell_sec}s, return_home={self.return_home}')

    # ================= 给 app 前端的 HTTP 口 =================

    def _start_http(self):
        port = int(self.get_parameter('http_port').value)
        try:
            self.httpd = ThreadingHTTPServer(('0.0.0.0', port), _TaskHttpHandler)
        except OSError as exc:
            self.httpd = None
            self.get_logger().error(
                f'HTTP :{port} 起不来（{exc}）—— app 上的任务卡会一直空白。'
                f'多半是端口被占，或上一个 task_manager 没退干净')
            return
        self.httpd.node = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever,
                                            daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'HTTP 已监听 :{port} —— GET /task/status')

    def _drain_http_cmds(self):
        """把 HTTP 线程塞进来的命令拿到 ROS 线程里执行（状态机只在这一个线程动）。"""
        while self.pending_cmds:
            cmd = self.pending_cmds.popleft()
            if cmd == '/task/skip':
                self.get_logger().info('app 点了「我倒完了」')
                self._on_cancel(Empty())
            elif cmd == '/task/cancel_all':
                self._on_cancel_all(Empty())
            elif cmd == '/task/force_resume':
                self._force_resume()

    # ================= 低电归位 =================
    #
    # 三道保险，缺一条这功能就不能上实机：
    #   1. critical 地板 —— 电压塌到底还在走廊里跑，遇到的是电机堵转 / 欠压复位 /
    #      TF 断裂，比停在原地糟得多。所以到底了就【就地停】，连归位都不试。
    #   2. 失效开放 —— /battery_status 失联或 present=false 时【绝不触发低电】。
    #      反过来做的话，一次固件回滚就能把机器人永久锁在 HOME。
    #   3. 人工覆盖 —— 现场演示时 20% 也得能跑，否则用户会自己把这功能关掉。

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_battery(self, msg: BatteryState):
        self.batt_stamp = self._now()
        self.batt_present = bool(msg.present)
        v = float(msg.voltage)
        self.batt_v = v if math.isfinite(v) else None
        p = float(msg.percentage)
        self.batt_pct = int(round(p * 100.0)) if math.isfinite(p) else None

    def _pct_text(self) -> str:
        return f'{self.batt_pct}%' if self.batt_pct is not None else '未知'

    def _check_battery(self):
        """1 Hz。所有阈值每次重读参数，`ros2 param set` 当场生效。"""
        now = self._now()
        stale_sec = float(self.get_parameter('battery_stale_sec').value)

        never_seen = self.batt_stamp is None
        stale = (never_seen
                 or self.batt_v is None
                 or not self.batt_present
                 or now - self.batt_stamp > stale_sec)

        if stale and not self.battery_stale:
            # 真掉线（之前有数据）才是异常，值得 warn
            self.get_logger().warn(
                f'/battery_status 失联超过 {stale_sec:.0f}s（或 present=false）—— '
                f'低电判定已暂停（失效开放），导航不受影响'
                + ('；当前低电锁定仍生效，需要放行请 POST /task/force_resume'
                   if self.low_battery else ''))
        elif self.battery_stale and not stale:
            self.get_logger().info(
                f'/battery_status 接上了：{self.batt_v:.2f}V / {self._pct_text()} '
                f'—— 低电保护生效')
        self.battery_stale = stale

        if stale:
            # 去抖计时器清零：失联期间的时间不该算进"连续低于阈值"。
            # ⚠️ 已经置位的 low_battery 【保持不变】—— 数据丢了不是"电量回来了"，
            #    不能因此让一台真的没电的机器人重新接单。要放行走 force_resume。
            self.low_since = self.resume_since = None
            if never_seen:
                # 仿真、或还没接电池的台架就是这个状态，不是故障 —— info 且低频，
                # 不要每分钟报一次警把真正的告警淹了。
                self.get_logger().info(
                    '还没收到过 /battery_status —— 低电保护未激活（失效开放）',
                    throttle_duration_sec=300.0)
            return

        if self.override_until is not None:
            if now < self.override_until:
                return                      # 覆盖期内不做任何低电判定
            self.override_until = None
            self.get_logger().warn('人工覆盖到期 —— 低电保护恢复')

        v = self.batt_v

        # ---- 保险 1：critical 物理地板 ----
        if v <= float(self.get_parameter('critical_voltage').value):
            if not self.battery_critical:
                self.battery_critical = True
                self.low_battery = True
                self.last_error = f'电量极低（{v:.2f}V / {self._pct_text()}），已就地停机'
                self.get_logger().error(
                    f'{self.last_error} —— 不再尝试归位，请立即充电')
                self._enter_critical()
            return
        self.battery_critical = False

        # ---- 低电置位 / 解除 ----
        if not self.low_battery:
            if v <= float(self.get_parameter('low_voltage').value):
                if self.low_since is None:
                    self.low_since = now
                elif now - self.low_since >= float(
                        self.get_parameter('low_debounce_sec').value):
                    self._enter_low_battery()
            else:
                self.low_since = None
        else:
            if v >= float(self.get_parameter('resume_voltage').value):
                if self.resume_since is None:
                    self.resume_since = now
                elif now - self.resume_since >= float(
                        self.get_parameter('resume_debounce_sec').value):
                    self._exit_low_battery(f'电压回到 {v:.2f}V 并保持')
            else:
                self.resume_since = None

    def _enter_low_battery(self):
        """置位低电 → 清空队列 → 复用已验证的 cancel 语义把当前这件事结束掉。

        刻意【不新写状态机】：_cancel_active() → _on_result(CANCELED) → _advance()
        这条路已经被"归位途中被召唤"用了很久，是验证过的。低电只是往里塞一个
        "队列是空的、而且到家之后要停 STOPPED 而不是 IDLE"的前提。
        """
        self.low_battery = True
        self.low_since = self.resume_since = None

        dropped = [t['name'] for t in self.queue]
        self.queue.clear()

        why = f'电量低（{self._pct_text()}，{self.batt_v:.2f}V），已放弃任务并归位'
        if dropped:
            why += f'；丢弃 {len(dropped)} 个待办：{"、".join(dropped)}'
        self.last_error = why
        self.get_logger().warn(why)

        if self.state == NAVIGATING:
            self._cancel_active()
        elif self.state == WAITING:
            self._clear_dwell()
            self._advance()
        elif self.state in (IDLE, STOPPED):
            # 已经在家就直接停住；不在家（比如被人推走了）尝试一次归位
            self._advance()
        # RETURNING：本来就在回家路上，不动作 —— 到达时 _on_result 会进 STOPPED
        self._publish_status()

    def _enter_critical(self):
        """就地停。和 _enter_low_battery 的区别是【不归位】。

        取消当前目标后 _advance() 会看到 battery_critical 为真而直接进 STOPPED，
        不会再发出新的 NavigateToPose goal。
        """
        self.queue.clear()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._clear_retry()
        self._clear_dwell()
        self._enter(STOPPED)

    def _exit_low_battery(self, reason: str):
        self.low_battery = False
        self.battery_critical = False
        self.low_since = self.resume_since = None
        self.last_error = ''
        self.get_logger().info(f'低电解除（{reason}）—— 恢复接单')
        # 状态留在 STOPPED：新召唤进来时 _on_goal 会看到 state in (IDLE, STOPPED)
        # 而调 _advance()，和"重试耗尽后停住"的恢复路径完全一样。
        self._publish_status()

    def _force_resume(self):
        """人工覆盖：override_sec 内【完全不做】低电判定，critical 地板也一并绕过。

        绕过 critical 是有意的：地板触发时机器人可能正停在门口或走廊中央，
        需要有办法把它挪走。这是一个人看着电压做出的显式决定，
        代价（可能半路欠压停机）写在日志里，不藏着。
        """
        sec = float(self.get_parameter('override_sec').value)
        self.override_until = self._now() + sec
        v = f'{self.batt_v:.2f}V' if self.batt_v is not None else '电压未知'
        self.get_logger().warn(
            f'⚠️ 人工覆盖低电保护 {sec:.0f}s（当前 {v} / {self._pct_text()}）—— '
            f'低电与 critical 地板都不再判定，电量确实不足时机器人可能在半路停下')
        self._exit_low_battery(f'人工覆盖 {sec:.0f}s')

    def _idle_state(self) -> str:
        """队列做空、已在待命点时进哪个状态。

        低电时必须是 STOPPED 而不是 IDLE：IDLE 的语义是"待命，随时可以接单"，
        而前端也只在 STOPPED 时显示 last_error —— 进 IDLE 的话用户既看不到
        "为什么停了"，界面上任务卡还会整个消失（shouldShowCard => !isIdle）。
        """
        return STOPPED if (self.low_battery or self.battery_critical) else IDLE

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

        # 低电时【拒绝，不排队】。排队意味着"充满电后几小时突然自己跑起来"，
        # 对真实使用是惊吓不是便利。写 last_error 是为了让用户看到原因 ——
        # 前端只在 STOPPED 时显示它，而低电时状态正好就是 STOPPED。
        if self.low_battery:
            self.last_error = f'电量低（{self._pct_text()}），已拒绝召唤「{name}」'
            self.get_logger().warn(self.last_error)
            self._publish_status()
            return

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

        # 电压塌到地板了 —— 就地停，连回家都不试（保险 1）。
        # 放在最前面是因为 _cancel_active() 的 CANCELED 回调也会走到这里，
        # 不拦住的话取消完当前目标又会立刻发一个归位目标出去。
        if self.battery_critical:
            self.queue.clear()
            self._enter(STOPPED)
            return

        # 兜底：低电期间队列必须是空的。正常路径下 _enter_low_battery() 已经清过，
        # 这里防的是"清空之后、状态落定之前又进来一个召唤"的竞态。
        if self.low_battery and self.queue:
            self.get_logger().warn(f'低电期间队列复活了 {len(self.queue)} 项 —— 再清一次')
            self.queue.clear()

        if self.queue:
            task = self.queue.popleft()
            self._goto(task, 'task')
            return

        if not self.return_home:
            self._enter(self._idle_state())
            return

        home = self._home()
        if home is None:
            self.get_logger().warn('没有待命点可回（TF 也拿不到）—— 原地待命')
            self._enter(self._idle_state())
            return
        if self._at(home):
            self._enter(self._idle_state())
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
        self.distance_remaining = 0.0
        self.eta_sec = 0.0            # 新目标 → ETA 重新起算，别用上一段的平滑值
        self._enter(RETURNING if kind == 'home' else NAVIGATING)
        self._send_nav_goal(task)

    def _enter(self, state):
        if state in (IDLE, STOPPED):
            self.current = None
            self.active_kind = None
            self.goal_handle = None
        if state in (IDLE, STOPPED, WAITING):
            self.distance_remaining = 0.0
            self.eta_sec = 0.0
        self.state = state
        self._publish_status()

    # ================= NavigateToPose =================

    def _send_nav_goal(self, task):
        # critical 的最后一道闸：_advance() 已经拦过，但 retry_timer 可能在电压塌下去
        # 之前就排上了，那条路绕过 _advance() 直接到这里。地板的语义是"不再发出
        # 任何导航目标"，所以这里必须再挡一次。
        if self.battery_critical:
            self.get_logger().error('电量已到临界地板 —— 不再发出导航目标')
            self._enter(STOPPED)
            return

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
        self.nav.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_accepted)

    def _on_feedback(self, msg):
        """Nav2 的导航反馈 → 剩余距离与 ETA。

        ⚠️ 刻意【不用】feedback 里的 estimated_time_remaining：Nav2 算的是
        "剩余距离 ÷ 当前瞬时线速度"，机器人一转弯/过门减速，线速度趋近 0，
        那个值就炸到几百秒再跳回来 —— 显示出来像坏了。

        distance_remaining 是剩余【路径长度】，很稳，只用它；ETA 自己按
        eta_speed_mps 折算，再做一次指数平滑，免得随路径重规划抖动。
        """
        fb = msg.feedback
        self.distance_remaining = float(fb.distance_remaining)
        raw = self.distance_remaining / self.eta_speed
        # 首帧直接取值，之后 EMA。系数 0.3：够跟手，又不会一抖一跳
        self.eta_sec = raw if self.eta_sec <= 0.0 else 0.7 * self.eta_sec + 0.3 * raw

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
                if self.low_battery:
                    self.last_error = (
                        f'电量低（{self._pct_text()}），已回到待命点，'
                        f'充电后自动恢复')
                    self.get_logger().warn(f'{self.last_error} —— 进 STOPPED，拒绝新召唤')
                    self._enter(STOPPED)
                else:
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
        # 每次进等待都【重新读】参数，而不是用 __init__ 里缓存的值 ——
        # 这样 `ros2 param set /task_manager dwell_sec 180.0` 能当场生效，调参不用重启。
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
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
        now = self._now()
        remaining = 0.0
        if self.state == WAITING and self.dwell_deadline is not None:
            remaining = max(0.0, self.dwell_deadline - now)

        override_left = 0.0
        if self.override_until is not None:
            override_left = max(0.0, self.override_until - now)

        self.status_json = json.dumps({
            'state': self.state,
            'current': self.current,
            'queue': [t['name'] for t in self.queue],
            'queue_len': len(self.queue),
            'dwell_remaining': round(remaining, 1),
            # 前端会同时显示这两个：distance 是 Nav2 给的真实路径长度，eta 是我们
            # 折算的估计值。ETA 不准时看 distance 就能分清"估歪了"还是"真卡住了"。
            'distance_remaining': round(self.distance_remaining, 2),
            'eta_sec': round(self.eta_sec, 1),
            'last_error': self.last_error,
            # 电量。state 取值【不变】（仍是原五个）—— 低电时是 STOPPED，
            # 靠 battery_low 区分是"重试耗尽/被取消"还是"没电了"。
            # 无数据时发 null 而不是 0：0 会被读成"电量为零"，比"不知道"更糟。
            'battery_pct': self.batt_pct,
            'battery_v': round(self.batt_v, 2) if self.batt_v is not None else None,
            'battery_low': self.low_battery,
            'battery_critical': self.battery_critical,
            'battery_stale': self.battery_stale,
            'battery_override_remaining': round(override_left, 1),
            'stamp': round(now, 3),
        }, ensure_ascii=False)

        msg = String()
        msg.data = self.status_json    # HTTP 线程读的也是这一份快照
        self.status_pub.publish(msg)

    def destroy_node(self):
        if getattr(self, 'httpd', None) is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
        super().destroy_node()


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
