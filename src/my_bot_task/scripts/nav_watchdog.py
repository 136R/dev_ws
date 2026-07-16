#!/usr/bin/env python3
"""导航卡死看门狗：兜底"Nav2 整个挂起、连失败状态都不返回"的情况。

task_manager 的重试（_fail()）只在 Nav2 明确回 ABORTED/CANCELED 时触发 —— 如果
controller_server/bt_navigator 挂起（不崩溃，但也不再回任何终止状态），task_manager
会在 NAVIGATING/RETURNING 里【永远等下去】，队列卡住且没有任何报错（`last_error`
不会被写，因为 _fail() 根本没被调用）。

本节点独立于 Nav2 之外，只看两样已经存在的东西：
    /task/status       —— task_manager 已经在发的 latched JSON（state / current）
    /odometry/filtered —— EKF 输出的机器人真实位姿

规则很简单：state 停在 NAVIGATING/RETURNING 期间，机器人的位置连续
stuck_timeout_sec 都没挪动 min_progress_m 以上 —— 判定卡死。

卡死后【不】尝试去够 Nav2 内部的恢复行为（那是它自己 BT 里 RecoveryFallback 的职责，
本节点够不着也没必要够，且默认阈值远大于那套恢复动作的正常耗时，不会撞车）。
只发一个 /goal_pose/cancel，复用 task_manager 已经验证过的 cancel 语义：
    NAVIGATING 时 = 取消当前任务、丢弃、去下一个
    RETURNING  时 = 取消归位、进 STOPPED
这样卡死至少会被"显式地结束"，而不是无限期静默挂起。
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

ACTIVE_STATES = ('NAVIGATING', 'RETURNING')


class NavWatchdog(Node):
    def __init__(self):
        super().__init__('nav_watchdog')

        self.declare_parameter('stuck_timeout_sec', 30.0)
        self.declare_parameter('min_progress_m', 0.15)
        self.declare_parameter('check_period_sec', 2.0)

        # ⚠️ 不缓存进 __init__ —— stuck_timeout/min_progress 每次 _check() 都重新读
        # 参数服务器（见下）。这样 `ros2 param set /nav_watchdog stuck_timeout_sec 10.0`
        # 能当场生效，调参/测试不用重启节点。check_period 只在起定时器这一刻用一次，
        # 改它必须重启（Humble 的 rclpy 定时器周期没法热改），所以它可以缓存。
        check_period = float(self.get_parameter('check_period_sec').value)

        self.task_state = ''
        self.task_name = ''
        self.pose = None            # (x, y)，来自 /odometry/filtered
        self.baseline_pose = None   # 上一次"确认有进展"时的位置
        self.baseline_time = None   # 对应的时间戳
        self.triggered = False      # 这一段 NAVIGATING/RETURNING 是否已经发过 cancel

        # 和 /task/status 同款 QoS：TRANSIENT_LOCAL 才能收到"已经发过的最后一条"，
        # 不然节点晚起一步就得等下一次状态变化才有数据。
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/nav_watchdog/status', latched)
        self.cancel_pub = self.create_publisher(Empty, '/goal_pose/cancel', 10)

        self.create_subscription(String, '/task/status', self._on_task_status, latched)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_timer(check_period, self._check)

        self._publish_status('IDLE', 0.0)
        self.get_logger().info(
            f'看门狗就绪：{self._stuck_timeout():.0f}s 内位移 < {self._min_progress():.2f}m 判定卡死')

    def _stuck_timeout(self) -> float:
        return float(self.get_parameter('stuck_timeout_sec').value)

    def _min_progress(self) -> float:
        return float(self.get_parameter('min_progress_m').value)

    def _on_task_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = data.get('state', '')
        current = data.get('current') or {}
        name = current.get('name', '')

        entering_active = state in ACTIVE_STATES and self.task_state not in ACTIVE_STATES
        changed_target = state in ACTIVE_STATES and name != self.task_name
        if entering_active or changed_target:
            self._reset_baseline()
        if state not in ACTIVE_STATES:
            self.triggered = False

        self.task_state = state
        self.task_name = name

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _reset_baseline(self):
        self.baseline_pose = self.pose
        self.baseline_time = self._now()
        self.triggered = False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _check(self):
        self._check_stuck()

    def _check_stuck(self):
        if self.task_state not in ACTIVE_STATES or self.pose is None:
            return
        if self.baseline_pose is None or self.baseline_time is None:
            self._reset_baseline()
            return

        min_progress = self._min_progress()
        moved = math.hypot(self.pose[0] - self.baseline_pose[0],
                           self.pose[1] - self.baseline_pose[1])
        if moved >= min_progress:
            self._reset_baseline()
            self._publish_status('OK', 0.0)
            return

        stalled_for = self._now() - self.baseline_time
        self._publish_status('WATCHING', stalled_for)
        stuck_timeout = self._stuck_timeout()
        if stalled_for < stuck_timeout or self.triggered:
            return

        self.triggered = True
        self.get_logger().error(
            f'「{self.task_name or "?"}」{stuck_timeout:.0f}s 内位移 < '
            f'{min_progress:.2f}m —— 判定卡死，发 /goal_pose/cancel 让任务层放弃')
        self._publish_status('STUCK', stalled_for)
        self.cancel_pub.publish(Empty())

    def _publish_status(self, status, stalled_for):
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'task_state': self.task_state,
            'task_name': self.task_name,
            'stalled_for_sec': round(stalled_for, 1),
            'stamp': round(self._now(), 3),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavWatchdog()
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
