#!/usr/bin/env python3
"""电压 → 电量百分比：/battery_raw（2 Hz 原始）→ /battery_status（1 Hz 平滑）。

【为什么要有这个节点，而不是让 my_bot_hw 直接发百分比】

低电判定走【电压】，电量显示走【百分比】—— 这两条路必须分开，否则误差会串联：

    电压 ──→ 决策（task_manager 的 low/critical 阈值，要求精确）
      └──→ 百分比 ──→ GUI 显示（用户已明确 ±10% 即可）

若"要不要回家"由百分比决定、而百分比允许 ±10%，那么"20% 触发归位"的实际触发点会
漂到 10%~30%，和 critical 地板挤在一起。拆开后两条路互不污染：
task_manager 订的是本节点发的 `voltage`（滤波后），`percentage` 只喂 GUI。

【滤波为什么不能省】

精度放宽了，但噪声一点没变小。3S 锂电的平台区从 80% 掉到 20% 只有约 0.6 V，
±10% 对应的电压分辨率约 0.1 V —— 而上一卡实测行驶时读数峰峰值就有 0.20 V 量级。
**噪声比信号大**。所以处理链的顺序是固定的，不能改：

    越界剔除 → 15 点中值（@2 Hz ≈ 7.5 s，杀电机电流尖峰）
             → EMA τ≈30 s（压住剩余的慢抖）
             → 查表线性插值 → raw_pct
             → 单调棘轮（发布值可自由下降，上升要连续 180 s 才放行）
             → 夹到 [1, 100]

中值在前、EMA 在后：中值杀的是"尖峰"这种离群点，EMA 杀的是"抖动"这种带内噪声。
反过来的话尖峰会先被 EMA 抹进历史里，再也剔不掉。

棘轮专治"机器人一停下来电压回弹、界面电量往上跳" —— 那是内阻压降恢复，不是充电。

【percentage 下限夹到 1%，绝不发 0】

GUI 后端 `node_streamer.cpp` 的 OnBattery 只转发 percentage，而 Dart 侧
`ws_channel.dart` 是 `p > 0 ? p : 旧值` —— **发 0 会被前端静默忽略，界面显示陈旧值**。
真没电了显示 1% 是对的，显示"上一次的 40%"是错的。

【表为什么是平行的两个数组，而不是 [[v, pct], ...]】

ROS 2 参数类型只有标量和一维数组，**不支持嵌套列表**。写成 `- [12.60, 100]` 的话
rclpy 在 declare_parameter 阶段就会抛类型错误。所以拆成 voltage_pct_table_v /
voltage_pct_table_pct 两个等长数组，启动时校验长度与单调性。
"""

import bisect
import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# 合理电压窗口。超出即判坏帧丢弃（不进滤波器）——
# 分压量程上限是 18.8 V，低于 5 V 说明不是电池而是采样/协议出了问题。
VOLTAGE_MIN_V = 5.0
VOLTAGE_MAX_V = 20.0

# 多久没有【有效】原始帧就停止发布 /battery_status。
# 刻意不做成参数：输入固定 2 Hz，3 s = 连丢 6 帧，这不是需要现场调的东西。
# 停止发布之后由 task_manager 的 battery_stale_sec 接手判定失效开放 ——
# 那一层才是"多久算失联"该配置的地方，两处都可配只会让人不知道该调哪个。
RAW_STALE_SEC = 3.0


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('voltage_pct_table_v', [12.60, 11.85, 11.10, 10.50, 9.60])
        self.declare_parameter('voltage_pct_table_pct', [100.0, 75.0, 40.0, 15.0, 0.0])
        self.declare_parameter('median_window', 15)
        self.declare_parameter('ema_tau_sec', 30.0)
        self.declare_parameter('ratchet_rise_hold_sec', 180.0)
        self.declare_parameter('debug_force_voltage', -1.0)

        self.table_v = [float(x) for x in self.get_parameter('voltage_pct_table_v').value]
        self.table_p = [float(x) for x in self.get_parameter('voltage_pct_table_pct').value]
        self._validate_table()

        self.median_window = max(1, int(self.get_parameter('median_window').value))
        self.ema_tau = max(0.1, float(self.get_parameter('ema_tau_sec').value))
        self.rise_hold = max(0.0, float(self.get_parameter('ratchet_rise_hold_sec').value))
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))

        # ---- 处理链状态 ----
        self.window = deque(maxlen=self.median_window)
        self.ema_v = None             # EMA 后的电压 —— 这个才是决策用的值
        self.ema_stamp = None
        self.pub_pct = None           # 棘轮之后、真正发布的百分比
        self.rise_since = None        # 连续"想上升"的起始时刻；None = 当前不在上升
        self.status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.last_good = None         # 最后一次收到有效样本的时刻
        self.bad_frames = 0
        self.forcing = False          # 上一 tick 是否处于注入模式（只为切换时打日志）

        self.pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self.create_subscription(BatteryState, '/battery_raw', self._on_raw, 10)

        # 注入模式要能【脱离真实数据自驱动】—— A4 的仿真验收里根本没有 STM32。
        # 所以按输入的标称速率 2 Hz 自己造样本，喂进同一条处理链，
        # 中值窗口的时间跨度和 EMA 的时间常数才和实机一致。
        self.create_timer(0.5, self._force_tick)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'电量监视就绪：中值 {self.median_window} 点 / EMA τ={self.ema_tau:.0f}s / '
            f'棘轮 {self.rise_hold:.0f}s，表 '
            + ' → '.join(f'{v:.2f}V={p:.0f}%' for v, p in zip(self.table_v, self.table_p)))

    # ================= 电压→百分比 表 =================

    def _validate_table(self):
        """表必须成对、至少两点、电压与百分比【同时严格递减】。

        不单调的表会让 bisect 查出错误的区间，插值结果毫无征兆地乱跳 ——
        这种错必须在启动时就炸掉，不能带着跑到实机上。
        """
        why = None
        if len(self.table_v) != len(self.table_p):
            why = (f'voltage_pct_table_v 有 {len(self.table_v)} 项、'
                   f'voltage_pct_table_pct 有 {len(self.table_p)} 项 —— 两个数组必须等长')
        elif len(self.table_v) < 2:
            why = f'表只有 {len(self.table_v)} 个点，至少要 2 个才能插值'
        else:
            for i in range(len(self.table_v) - 1):
                if self.table_v[i] <= self.table_v[i + 1]:
                    why = (f'电压列在第 {i} → {i + 1} 项不是严格递减：'
                           f'{self.table_v[i]} → {self.table_v[i + 1]}')
                    break
                if self.table_p[i] <= self.table_p[i + 1]:
                    why = (f'百分比列在第 {i} → {i + 1} 项不是严格递减：'
                           f'{self.table_p[i]} → {self.table_p[i + 1]}')
                    break
        if why is None:
            return
        self.get_logger().error(
            f'battery.yaml 的电压→电量表非法：{why}。'
            f'表必须按电压【从高到低】排列，百分比同步递减。节点退出。')
        raise SystemExit(1)

    def _lookup_pct(self, v: float) -> float:
        """线性插值。超出表的两端直接夹到端点值。"""
        if v >= self.table_v[0]:
            return self.table_p[0]
        if v <= self.table_v[-1]:
            return self.table_p[-1]
        # 表是递减的，bisect 要升序 —— 在反转后的视图上找
        asc = self.table_v[::-1]
        j = bisect.bisect_left(asc, v)          # asc[j-1] < v <= asc[j]
        hi = len(self.table_v) - 1 - j          # 高电压侧下标
        lo = hi + 1                             # 低电压侧下标
        span = self.table_v[hi] - self.table_v[lo]
        t = (v - self.table_v[lo]) / span
        return self.table_p[lo] + t * (self.table_p[hi] - self.table_p[lo])

    # ================= 采样 =================

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _force_voltage(self):
        """>0 时旁路真实数据。返回 None 表示不在注入模式。"""
        v = float(self.get_parameter('debug_force_voltage').value)
        return v if v > 0.0 else None

    def _force_tick(self):
        forced = self._force_voltage()
        if forced is None:
            if self.forcing:
                self.get_logger().warn('debug_force_voltage 已关闭 → 恢复使用 /battery_raw')
                self.forcing = False
            return
        if not self.forcing:
            self.get_logger().warn(
                f'⚠️ debug_force_voltage={forced:.2f}V 已生效 —— '
                f'真实 /battery_raw 被旁路。实机上跑完记得关掉（设回 -1.0）')
            self.forcing = True
        self._ingest(forced)

    def _on_raw(self, msg: BatteryState):
        if self.forcing:
            return                     # 注入模式下真实数据整条丢弃
        # present=false 意味着固件 proto_ver 对不上或 voltage_valid=0 ——
        # 那个电压是拿来排障的，不是拿来做决策的，不能进滤波器。
        if not msg.present:
            self.get_logger().warn(
                '/battery_raw 的 present=false（多半是固件 proto_ver 不匹配）—— 已丢弃',
                throttle_duration_sec=30.0)
            return
        self._ingest(float(msg.voltage))

    def _ingest(self, v: float):
        if not math.isfinite(v) or not (VOLTAGE_MIN_V <= v <= VOLTAGE_MAX_V):
            self.bad_frames += 1
            self.get_logger().warn(
                f'电压 {v:.2f}V 越界（{VOLTAGE_MIN_V}~{VOLTAGE_MAX_V}V）—— '
                f'已丢弃，累计 {self.bad_frames} 帧', throttle_duration_sec=10.0)
            return

        now = self._now()
        self.window.append(v)
        med = sorted(self.window)[len(self.window) // 2]

        if self.ema_v is None:
            self.ema_v = med           # 首个样本直接落地，不从 0 爬 30 秒
        else:
            dt = max(0.0, now - self.ema_stamp)
            # 按实际间隔算系数，而不是固定 alpha —— 掉几帧时时间常数才不会走样
            alpha = 1.0 - math.exp(-dt / self.ema_tau)
            self.ema_v += alpha * (med - self.ema_v)
        self.ema_stamp = now
        self.last_good = now

    # ================= 棘轮 + 发布 =================

    def _ratchet(self, raw_pct: float, now: float) -> float:
        if self.pub_pct is None:
            self.pub_pct = raw_pct     # 开机第一帧就是真值，不然要爬 180 秒才对
            return self.pub_pct

        if raw_pct <= self.pub_pct:
            # 下降无条件放行：电量往下掉是真实的，压着不报只会误导用户
            self.pub_pct = raw_pct
            self.rise_since = None
            self.status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            return self.pub_pct

        # 想上升 —— 必须【连续】高于当前发布值 rise_hold 秒。中途跌回去就重新计时
        # （上面那个分支会把 rise_since 清成 None）。
        if self.rise_since is None:
            self.rise_since = now
        if now - self.rise_since >= self.rise_hold:
            if self.status != BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                self.get_logger().info(
                    f'电量持续回升 {self.rise_hold:.0f}s —— 判定为充电中，解除棘轮')
            self.pub_pct = raw_pct
            self.status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        return self.pub_pct

    def _publish(self):
        now = self._now()
        if self.last_good is None or now - self.last_good > RAW_STALE_SEC:
            # 没有可信数据时【什么都不发】，而不是发个 NaN 或 0：
            #   发 0   → 前端 `p > 0` 判定会静默忽略，界面停在陈旧值
            #   发 NaN → GUI 后端 OnBattery 会对 NaN 做整数转换，是未定义行为
            # 静默期由 task_manager 的 battery_stale_sec 超时接手 → 失效开放。
            return

        raw_pct = self._lookup_pct(self.ema_v)
        pct = self._ratchet(raw_pct, now)
        pct = min(100.0, max(1.0, pct))    # 下限 1%，绝不发 0（见模块注释）

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.ema_v)     # ← task_manager 的低电判定读的是这个
        msg.percentage = round(pct) / 100.0
        msg.present = True
        msg.power_supply_status = self.status
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        nan = float('nan')
        msg.temperature = nan
        msg.current = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan

        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = BatteryMonitor()
    except SystemExit as exc:
        rclpy.shutdown()
        sys.exit(exc.code)
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
