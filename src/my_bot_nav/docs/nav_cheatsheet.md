# my_bot_nav 命令速查表

> 适用包：`my_bot_nav`
> 工作空间：`~/dev_ws`

## 编译

```bash
cd ~/dev_ws
colcon build --packages-select my_bot_nav --symlink-install
source install/setup.bash
```

## 启动 Nav2

### 实机

```bash
ros2 launch my_bot_nav nav.launch.py
```

### 仿真

```bash
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
```

## 组合启动

```bash
ros2 launch my_bot_slam nav_slam.launch.py slam_mode:=localization
ros2 launch my_bot_slam nav_slam.launch.py use_sim_time:=true slam_mode:=localization
```

## 发送导航目标

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

## 禁行区 / Keepout（方案一：app 画笔 → Nav2 Costmap Filter）

不改 ROS_Flutter_Gui_App 源码。app 后端把"原图+画笔"持久化到磁盘
`~/.maps/<map>/map.pgm`（前端走 HTTP 瓦片，不发 ROS 话题）。`keepout_mask_diff`
节点轮询该 PGM，与 slam_toolbox 实时 `/map` 做差，得到"用户新增的占据格"，
发布成独立掩码 `/keepout_filter_mask`，由 Nav2 的 `KeepoutFilter` 消费。
slam_toolbox 定位不受影响。

**关键依赖（必须改 app 配置一次）**：app 默认订阅 `/map`，会每 2s 把 slam 的
实时地图镜像回磁盘，**抹掉画笔**。因此把 app 的 `cfg/config.yaml` 改为
`sub_map_topic: "/map_gui"`；`keepout_mask_diff` 会在拿到第一帧 `/map` 后，
**一次性**（latched）转发到 `/map_gui`，app 只镜像一次、之后画笔持久保留、可增可删。
启动顺序：先 slam+nav，再 keepout（开始转发 /map_gui），最后启动 app。

仿真测试启动顺序：

```bash
# 1. 仿真 + slam_toolbox 定位 + Nav2(RPP)
ros2 launch my_bot launch_sim.launch.py         # 或你的仿真启动
ros2 launch my_bot_slam nav_slam.launch.py use_sim_time:=true slam_mode:=localization

# 2. 禁行区流水线（diff 节点 + costmap_filter_info_server + 生命周期管理）
ros2 launch my_bot_nav keepout.launch.py use_sim_time:=true

# 3. 启动 app 后端（它发布 /map_manager/map）
cd ~/ros_flutter_gui && sh ./start.sh           # 浏览器开 http://127.0.0.1:8080
```

在 app 里用画笔涂禁行区并保存后：

```bash
# 掩码是否生成（应能看到非 -1 的格子）
ros2 topic echo /keepout_filter_mask --once --field info
ros2 topic echo /costmap_filter_info --once
# 全局代价地图里禁行区应变为致命(lethal)
# RViz 订阅 /global_costmap/costmap 查看；或发导航目标看是否绕行
```

要点：
- 掩码语义：禁行格=100，其余=-1(未知)，KeepoutFilter 只叠加禁行区、不擦除静态层障碍。
- `keepout_filter` 放在 global/local costmap 的 `plugins` 里、`inflation_layer` 之前，
  因此禁行区会被膨胀（留出机器人间距）。
- 数据源是磁盘 PGM（默认 `~/.maps/map/map.yaml`），节点按 mtime 轮询，画笔保存后约 1s 生效；
  切换地图后改 `keepout.launch.py` 里 `edited_map_yaml` 指向对应目录即可。
- 没画任何禁行区时会发布"空掩码"（全 -1），这能消除 Nav2 的
  `KeepoutFilter: Filter mask was not received` 告警，属正常。
