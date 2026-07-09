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

## 参数文件

`slam.launch.py` 使用 `mode` 选择 SLAM Toolbox 节点：

- `mode:=mapping` 启动 `async_slam_toolbox_node`
- `mode:=localization` 启动 `localization_slam_toolbox_node`

`use_sim_time` 选择参数文件：

- `use_sim_time:=true` 使用 `config/mapper_params_sim.yaml`
- `use_sim_time:=false` 使用 `config/mapper_params_hw.yaml`

`mode` 和 `use_sim_time` 由 launch 覆盖，不需要手动改 YAML 里的同名字段。

## 保存地图

```bash
# 保存序列化地图 用于 localization 模式定位
ros2 service call /slam_toolbox/serialize_map \
slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/bingda/dev_ws/src/my_bot_slam/config/maps/hw/serialize_map/my_map'}"
```
