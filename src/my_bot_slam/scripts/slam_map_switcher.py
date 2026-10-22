#!/usr/bin/env python3
"""app 上切图后，定位跟着换图，不用重启。

轮询 ~/.maps/current_map，名字变了就调 /slam_toolbox/deserialize_map 加载新图的位姿图。
定位模式只接受 LOCALIZE_AT_POSE（别的类型会被拒："non-localization deserialization"）。
新图里车在哪不知道（slam_toolbox 没有全局重定位），先放在新图原点，等用户在 app 上重定位；
重定位之前叠进 /map 的几帧激光是错位的，/initialpose 一到就清掉。

只在定位模式、且没用 map:= 指定地图时由 slam.launch.py 启动。
"""

import os

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph

from map_resolve import MAPS_ROOT, localization_base


class SlamMapSwitcher(Node):
    def __init__(self):
        super().__init__('slam_map_switcher')
        self.declare_parameter('map_name', '')          # 启动时 slam 已加载的图
        self.declare_parameter('poll_period_sec', 2.0)
        self.loaded = self.get_parameter('map_name').value
        self.failed = None                             # 补不齐位姿图的图名，只报一次错
        self.inflight = False
        self.client = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.create_timer(float(self.get_parameter('poll_period_sec').value), self._poll)
        self.get_logger().info(f'当前地图 "{self.loaded}"，监听 app 切图')

    def _poll(self):
        if self.inflight:
            return
        try:
            with open(os.path.join(MAPS_ROOT, 'current_map')) as f:
                name = f.read().strip()
        except OSError:
            return
        if not name or name == self.loaded or name == self.failed:
            return
        try:
            base = localization_base(name)
        except RuntimeError as exc:
            self.failed = name
            self.get_logger().error(f'切到 "{name}" 失败，定位仍在 "{self.loaded}"：{exc}')
            return
        if not self.client.service_is_ready():
            self.get_logger().warn('deserialize_map 还没就绪，稍后重试', throttle_duration_sec=10.0)
            return
        req = DeserializePoseGraph.Request()
        req.filename = base
        req.match_type = DeserializePoseGraph.Request.LOCALIZE_AT_POSE
        self.inflight = True
        self.get_logger().info(f'app 切图 "{self.loaded}" → "{name}"，加载位姿图')
        self.client.call_async(req).add_done_callback(lambda f: self._done(f, name))

    def _done(self, future, name):
        self.inflight = False
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001 - 服务调用失败不该拖垮节点，下个周期重试
            self.get_logger().error(f'加载 "{name}" 失败: {exc}')
            return
        self.loaded, self.failed = name, None
        self.get_logger().warn(f'✔ 定位已切到 "{name}"，车暂定在新图原点 —— 请在 app 上重定位')


def main():
    rclpy.init()
    node = SlamMapSwitcher()
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
