# my_bot 仿真命令速查表

> 适用包：`my_bot`
> 工作空间：`~/dev_ws`

## 编译

```bash
cd ~/dev_ws
colcon build --packages-select my_bot --symlink-install
source install/setup.bash
```

## 启动仿真

```bash
ros2 launch my_bot launch_sim.launch.py
```

默认加载 `dynamic_indoor_crossing.sdf`，用于 NeuPAN 室内人员穿行动避验证。
需要切换旧场地时可覆盖 `world`：

```bash
ros2 launch my_bot launch_sim.launch.py world:=/home/bingda/dev_ws/src/my_bot/worlds/test_dynamic.sdf
ros2 launch my_bot launch_sim.launch.py world:=/home/bingda/dev_ws/src/my_bot/worlds/test_corridor.sdf
```

## 仿真常用调试

```bash
ros2 topic list
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

## 仿真配套功能

```bash
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
```

## NeuPAN 动态避障验证

场地：L 形室内通道，起点在 `(0, 0)` 附近，推荐目标点 `(3.0, 2.1)`。
两个移动障碍分别验证直线段停让和转角处绕行。

```bash
# 1. 启动仿真（默认加载 dynamic_indoor_crossing.sdf）
ros2 launch my_bot launch_sim.launch.py

# 2. 在线建图
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping

# 3. 启动 NeuPAN 局部规划栈
ros2 launch my_bot_nav neupan_sim.launch.py use_sim_time:=true

# 4. 发送目标
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.1, z: 0.0}, orientation: {w: 1.0}}}"

# 5. 记录指标（建议发目标后立即执行）
ros2 run my_bot_nav neupan_metrics.py --ros-args \
  -p goal_x:=3.0 -p goal_y:=2.1 -p arrive_radius:=0.2 -p timeout:=60.0
```

建议验收口径：

- 60 秒内到达目标 `0.2m` 范围。
- Gazebo 中无可见碰撞。
- `neupan_metrics.py` 的最小障碍距离建议不低于约 `0.15m`。
- 允许短暂停让，但不应长期卡死、持续原地抖动或反复大幅转向。
