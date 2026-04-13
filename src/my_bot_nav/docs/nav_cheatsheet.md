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
