# my_bot_slam 命令速查表

> 适用包：`my_bot_slam`
> 工作空间：`~/dev_ws`

## 编译

```bash
cd ~/dev_ws
colcon build --packages-select my_bot_slam --symlink-install
source install/setup.bash
```

## 实机建图

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

## 实机定位

```bash
ros2 launch my_bot_slam slam.launch.py mode:=localization
```

## 仿真 SLAM

```bash
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization
```

## 保存地图

```bash
# 保存序列化地图 用于 localization 模式定位
ros2 service call /slam_toolbox/serialize_map \
slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/orangepi/dev_ws/src/my_bot_slam/config/serialize_map/my_map'}"
```
