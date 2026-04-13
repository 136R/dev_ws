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
