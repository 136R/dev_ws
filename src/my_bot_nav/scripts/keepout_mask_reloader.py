#!/usr/bin/env python3
"""画笔改动后自动热加载 keepout 掩码。

filter_mask_server 只在启动时加载一次 yaml。app 上重新画完禁行区并保存后，
后端会重写 ~/.maps/<name>/<name>.pgm —— 本节点按 mtime 轮询发现它变了，
就调 /filter_mask_server/load_map 把新掩码热加载进去，不用重启任何节点。

轮询的是磁盘文件，所以只有"点了保存"才会触发；在网页上画的过程中磁盘没动，Nav2 不受影响。
"""

import os

import rclpy
from nav2_msgs.srv import LoadMap
from rclpy.node import Node


class KeepoutMaskReloader(Node):
    def __init__(self):
        super().__init__('keepout_mask_reloader')
        self.declare_parameter('mask_yaml', '')
        self.declare_parameter('poll_period_sec', 1.0)
        self.declare_parameter('service_name', '/filter_mask_server/load_map')
        # 保存写 pgm 可能分多次落盘，等文件静默这么久再加载，避免读到写了一半的图
        self.declare_parameter('settle_sec', 0.5)

        self.mask_yaml = self.get_parameter('mask_yaml').value
        if not self.mask_yaml:
            raise RuntimeError('必须指定 mask_yaml 参数')
        self.mask_pgm = os.path.splitext(self.mask_yaml)[0] + '.pgm'
        self.settle = float(self.get_parameter('settle_sec').value)

        self.client = self.create_client(
            LoadMap, self.get_parameter('service_name').value)

        # 启动时记下当前 mtime：filter_mask_server 已经加载过这一版，不要重复加载
        self.seen = self._mtime()
        self.pending = None          # 检测到改动的时刻，用于 settle 去抖
        self.inflight = False

        self.create_timer(
            float(self.get_parameter('poll_period_sec').value), self._poll)
        self.get_logger().info(f'监听画笔改动: {self.mask_pgm}')

    def _mtime(self) -> float:
        try:
            return max(os.path.getmtime(self.mask_pgm),
                       os.path.getmtime(self.mask_yaml))
        except OSError:
            return 0.0

    def _poll(self):
        if self.inflight:
            return
        now = self._mtime()
        if now == 0.0 or now == self.seen:
            return
        # 文件还在变 → 继续等它静下来
        if self.pending is None or self.pending != now:
            self.pending = now
            return
        if not self.client.service_is_ready():
            self.get_logger().warn(
                f'{self.client.srv_name} 还没就绪，稍后重试', throttle_duration_sec=10.0)
            return

        self.seen, self.pending, self.inflight = now, None, True
        self.get_logger().info('画笔有改动 → 热加载掩码')
        req = LoadMap.Request()
        req.map_url = self.mask_yaml
        self.client.call_async(req).add_done_callback(self._done)

    def _done(self, future):
        self.inflight = False
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001 - 服务调用失败不该拖垮节点
            self.get_logger().error(f'热加载失败: {exc}')
            return
        if result == LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().info('✔ 掩码已热加载，禁行区即时生效')
        else:
            self.get_logger().error(f'热加载失败，LoadMap result={result}')


def main():
    rclpy.init()
    node = KeepoutMaskReloader()
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
