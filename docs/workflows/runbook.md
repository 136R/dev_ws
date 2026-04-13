# 运行手册

本文档给出当前四包结构下最常用的启动方式：

- `my_bot`：通用模型与仿真
- `my_bot_hw`：实机底盘与硬件 bringup
- `my_bot_slam`：SLAM Toolbox
- `my_bot_nav`：Nav2

默认工作空间路径：`~/dev_ws`

---

## 一、通用准备

```bash
cd ~/dev_ws
source install/setup.bash
```

首次或代码更新后：

```bash
colcon build --packages-select my_bot my_bot_hw my_bot_slam my_bot_nav --symlink-install
source install/setup.bash
```

---

## 二、仿真

### 2.1 只启动 Gazebo 仿真

```bash
ros2 launch my_bot launch_sim.launch.py
```

### 2.2 仿真 + SLAM

终端 1：

```bash
ros2 launch my_bot launch_sim.launch.py
```

终端 2：

```bash
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
```

### 2.3 仿真 + Nav2

终端 1：

```bash
ros2 launch my_bot launch_sim.launch.py
```

终端 2：

```bash
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
```

### 2.4 仿真 + SLAM + Nav2

终端 1：

```bash
ros2 launch my_bot launch_sim.launch.py
```

终端 2：

```bash
ros2 launch my_bot_slam nav_slam.launch.py use_sim_time:=true slam_mode:=localization
```

---

## 三、实机

### 3.1 启动实机底盘

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

常用变体：

```bash
ros2 launch my_bot_hw robot_bringup.launch.py serial_port:=/dev/ttyS7
ros2 launch my_bot_hw robot_bringup.launch.py lidar_port:=/dev/ttyUSB0
```

### 3.2 实机建图

终端 1：

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

终端 2：

```bash
ros2 launch my_bot_slam slam.launch.py mode:=mapping
```

终端 3：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard
```

### 3.3 实机定位

终端 1：

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

终端 2：

```bash
ros2 launch my_bot_slam slam.launch.py mode:=localization
```

### 3.4 实机导航

终端 1：

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

终端 2：

```bash
ros2 launch my_bot_slam nav_slam.launch.py slam_mode:=localization
```

如果只想单独拉起 Nav2：

```bash
ros2 launch my_bot_nav nav.launch.py
```

---

## 四、常见组合建议

### 调模型

```bash
ros2 launch my_bot launch_sim.launch.py
```

### 调硬件驱动

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

### 调建图

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
ros2 launch my_bot_slam slam.launch.py mode:=mapping
```

### 调导航

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
ros2 launch my_bot_slam nav_slam.launch.py slam_mode:=localization
```
