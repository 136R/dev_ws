#!/usr/bin/env python3
"""
analyze_mppi.py — Read a rosbag2 and plot MPPI tracking behavior.
Usage: python3 analyze_mppi.py <bag_dir>
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_bag(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions('', '')
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = {
        'cmd_vx': [], 'cmd_wz': [], 'cmd_t': [],
        'smo_vx': [], 'smo_wz': [], 'smo_t': [],
        'odom_x': [], 'odom_y': [], 'odom_yaw': [],
        'odom_vx': [], 'odom_wz': [], 'odom_t': [],
        'plan_x': [], 'plan_y': [],            # 全局路径(取第一条)
    }
    plan_captured = False

    while reader.has_next():
        topic, raw, ts = reader.read_next()
        t = ts * 1e-9
        if topic not in type_map:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_type)

        if topic == '/cmd_vel':
            data['cmd_t'].append(t)
            data['cmd_vx'].append(msg.linear.x)
            data['cmd_wz'].append(msg.angular.z)

        elif topic == '/cmd_vel_smoothed':
            data['smo_t'].append(t)
            # TwistStamped
            data['smo_vx'].append(msg.twist.linear.x)
            data['smo_wz'].append(msg.twist.angular.z)

        elif topic == '/odometry/filtered':
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            # quaternion to yaw
            yaw = np.arctan2(2*(q.w*q.z + q.x*q.y),
                             1 - 2*(q.y*q.y + q.z*q.z))
            data['odom_t'].append(t)
            data['odom_x'].append(p.x)
            data['odom_y'].append(p.y)
            data['odom_yaw'].append(yaw)
            data['odom_vx'].append(msg.twist.twist.linear.x)
            data['odom_wz'].append(msg.twist.twist.angular.z)

        elif topic == '/plan' and not plan_captured:
            for pose in msg.poses:
                data['plan_x'].append(pose.pose.position.x)
                data['plan_y'].append(pose.pose.position.y)
            plan_captured = True

    return {k: np.array(v) for k, v in data.items()}

def plot(d):
    # 时间归零
    t0 = d['odom_t'][0] if len(d['odom_t']) else 0
    for k in ['cmd_t', 'smo_t', 'odom_t']:
        if len(d[k]):
            d[k] -= t0

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    # --- (1) XY 轨迹 ---
    ax = axes[0, 0]
    if len(d['plan_x']):
        ax.plot(d['plan_x'], d['plan_y'], 'g--', lw=2, label='/plan (global)')
    ax.plot(d['odom_x'], d['odom_y'], 'b-', lw=1.5, label='odom (actual)')
    # 起点终点
    ax.plot(d['odom_x'][0], d['odom_y'][0], 'go', ms=10, label='start')
    ax.plot(d['odom_x'][-1], d['odom_y'][-1], 'r*', ms=14, label='end')
    # 每隔 1s 画一个朝向箭头
    step = max(1, len(d['odom_t']) // 20)
    for i in range(0, len(d['odom_t']), step):
        ax.arrow(d['odom_x'][i], d['odom_y'][i],
                 0.05*np.cos(d['odom_yaw'][i]),
                 0.05*np.sin(d['odom_yaw'][i]),
                 head_width=0.02, color='b', alpha=0.4)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Trajectory (plan vs actual)')
    ax.axis('equal'); ax.grid(True); ax.legend()

    # --- (2) 线速度三链对比 ---
    ax = axes[0, 1]
    ax.plot(d['cmd_t'], d['cmd_vx'], 'r-',  lw=1, alpha=0.8, label='/cmd_vel (MPPI out)')
    ax.plot(d['smo_t'], d['smo_vx'], 'orange', lw=1, alpha=0.8, label='/cmd_vel_smoothed')
    ax.plot(d['odom_t'], d['odom_vx'], 'b-', lw=1, label='odom (actual)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('vx [m/s]')
    ax.set_title('Linear velocity chain')
    ax.grid(True); ax.legend()

    # --- (3) 角速度三链对比 ---
    ax = axes[1, 0]
    ax.plot(d['cmd_t'], d['cmd_wz'], 'r-',  lw=1, alpha=0.8, label='/cmd_vel (MPPI out)')
    ax.plot(d['smo_t'], d['smo_wz'], 'orange', lw=1, alpha=0.8, label='/cmd_vel_smoothed')
    ax.plot(d['odom_t'], d['odom_wz'], 'b-', lw=1, label='odom (actual)')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('t [s]'); ax.set_ylabel('wz [rad/s]')
    ax.set_title('Angular velocity chain  (直线应≈0)')
    ax.grid(True); ax.legend()

    # --- (4) MPPI 内部加速度（从 cmd_vel 差分） ---
    ax = axes[1, 1]
    if len(d['cmd_t']) > 1:
        dt = np.diff(d['cmd_t'])
        dt[dt < 1e-6] = 1e-6
        ax_cmd = np.diff(d['cmd_vx']) / dt
        az_cmd = np.diff(d['cmd_wz']) / dt
        ax.plot(d['cmd_t'][1:], ax_cmd, 'r-', lw=0.8, label='ax from /cmd_vel')
        ax.plot(d['cmd_t'][1:], az_cmd, 'm-', lw=0.8, label='az from /cmd_vel')
        ax.axhline(0.8,  color='g', ls='--', label='ax_max=0.8 (smoother)')
        ax.axhline(-0.8, color='g', ls='--')
        ax.axhline(1.5,  color='c', ls=':',  label='az_max=1.5 (smoother)')
        ax.axhline(-1.5, color='c', ls=':')
    ax.set_xlabel('t [s]'); ax.set_ylabel('accel')
    ax.set_title('MPPI commanded acceleration vs smoother limits')
    ax.grid(True); ax.legend()

    plt.tight_layout()
    plt.savefig('mppi_analysis.png', dpi=120)
    plt.show()
    print('Saved: mppi_analysis.png')

if __name__ == '__main__':
    bag = sys.argv[1] if len(sys.argv) > 1 else 'mppi_line_1m'
    data = read_bag(bag)
    print(f'cmd_vel samples:  {len(data["cmd_t"])}')
    print(f'smoothed samples: {len(data["smo_t"])}')
    print(f'odom samples:     {len(data["odom_t"])}')
    print(f'plan points:      {len(data["plan_x"])}')
    plot(data)