# robot_bringup.launch.py 话题说明

> 运行命令：`ros2 launch my_bot_hw robot_bringup.launch.py`
>
> 本文档介绍该 launch 文件启动后发布/订阅的所有 ROS2 话题，以及各话题的数据流向。

---

## 话题总览

| 话题 | 类型 | 方向 | 发布节点 | 订阅节点 |
|------|------|------|----------|----------|
| `/cmd_vel_keyboard` | `geometry_msgs/Twist` | 输入 | 外部（teleop） | twist_mux |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 输入 | 外部（Nav2） | twist_mux |
| `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/Twist` | 内部 | twist_mux | diff_cont |
| `/diff_cont/odom` | `nav_msgs/Odometry` | 内部 | diff_cont | odom_relay |
| `/odom` | `nav_msgs/Odometry` | 输出 | odom_relay | slam_toolbox / Nav2 |
| `/joint_states` | `sensor_msgs/JointState` | 内部 | joint_broad | robot_state_publisher |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | 内部 | joint_broad | — |
| `/robot_description` | `std_msgs/String` | 静态 | robot_state_publisher | ros2_control_node |
| `/scan` | `sensor_msgs/LaserScan` | 输出 | sllidar_node | slam_toolbox / Nav2 |
| `/tf` | `tf2_msgs/TFMessage` | 内部 | 多节点 | 全局 |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态 | robot_state_publisher | 全局 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 状态 | ros2_control_node | — |
| `/diff_cont/transition_event` | `lifecycle_msgs/TransitionEvent` | 状态 | diff_cont | — |
| `/joint_broad/transition_event` | `lifecycle_msgs/TransitionEvent` | 状态 | joint_broad | — |
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | 内部 | ROS2 框架 | — |
| `/rosout` | `rcl_interfaces/Log` | 日志 | 所有节点 | — |

---

## 速度指令流（cmd_vel 链路）

```
键盘遥控           Nav2
     │                │
/cmd_vel_keyboard  /cmd_vel_nav
(priority=90)      (priority=70)
     └──────┬─────────┘
          twist_mux
          （优先级仲裁，超时 0.5s 自动停止）
             │
/diff_cont/cmd_vel_unstamped
             │
        diff_cont
    （diff_drive_controller）
             │
   ┌─────────┴─────────┐
   left_wheel_joint    right_wheel_joint
   velocity command    velocity command
             │
   Stm32SerialHardware::write()
             │
   串口 /dev/ttyS7 → STM32F303
   发送 0x01 帧 [vx, 0, vyaw]（int16 × 1000，大端）
```

**速度限制（hw_controllers.yaml）：**
- 线速度：`-0.3 ~ +0.4 m/s`，加速度 ≤ 1.5 m/s²
- 角速度：`±1.2 rad/s`，加速度 ≤ 2.0 rad/s²

---

## 里程计流（odom 链路）

```
STM32F303
   │ 0x0A 帧（每 50Hz 查询）
   │ [vx_mm/s, vyaw_mrad/s, yaw_cdeg]（int16，大端）
   │
Stm32SerialHardware::read()
   │ 解算：v_left/right_rad_s = (vx ± vyaw * L/2) / R
   │ 积分：position += velocity * dt
   │
hardware_interface（left/right position & velocity states）
   │
diff_cont（diff_drive_controller）
   │ 融合轮速 → 计算底盘里程
   │ 发布 odom→base_footprint TF（50Hz）
   │
/diff_cont/odom（nav_msgs/Odometry，50Hz）
   │
odom_relay（topic_tools relay）
   │
/odom（nav_msgs/Odometry，50Hz）
   │
slam_toolbox / Nav2
```

**关键参数（hw_controllers.yaml）：**
- `wheel_separation: 0.171` m（轮距，中心到中心）
- `wheel_radius: 0.0325` m（轮半径 ~32.5mm）
- `publish_rate: 50.0` Hz

> ⚠️ `/diff_cont/odom` → `/odom` 的 relay 节点是必需的：`diff_drive_controller` 固定发布到 `/diff_cont/odom`，而 `slam_toolbox` 订阅 `/odom`，两者命名空间不同。

---

## 关节状态流（joint_states 链路）

```
Stm32SerialHardware（hardware_interface states）
   │ left_wheel_joint: position, velocity
   │ right_wheel_joint: position, velocity
   │
joint_broad（JointStateBroadcaster）
   │
/joint_states（sensor_msgs/JointState，~16.8Hz）
/dynamic_joint_states（control_msgs/DynamicJointState，~16.8Hz）
   │
robot_state_publisher
   │ 计算非驱动关节的 TF（caster_front/back_wheel 等）
   │
/tf_static（静态关节）
/tf（动态关节，随 /joint_states 更新）
```

---

## TF 坐标树

```
map                          ← slam_toolbox 启动后才存在
 └── odom                    ← diff_cont 发布（50Hz）
      └── base_footprint     ← diff_cont 发布（50Hz）
           └── base_link     ← robot_state_publisher（静态）
                └── chassis  ← robot_state_publisher（静态）
                     ├── laser_frame    ← 雷达安装位置（静态）
                     ├── caster_front_wheel（静态）
                     └── caster_back_wheel（静态）
                ├── left_wheel（随 /joint_states 更新）
                └── right_wheel（随 /joint_states 更新）
```

> `rate: 10000 / most_recent_transform: 0.000` 表示该帧为**静态 TF**（robot_state_publisher 通过 `/tf_static` 发布，latched），与动态频率无关，属正常现象。

---

## 传感器话题

### `/scan`（激光雷达）

| 字段 | 值 |
|------|----|
| 类型 | `sensor_msgs/LaserScan` |
| 发布节点 | `sllidar_node` |
| 频率 | 10 Hz（Standard 扫描模式） |
| 帧 ID | `laser_frame` |
| 串口 | `/dev/ttyUSB0`，波特率 460800 |
| 量程 | 最大 16.0 m（C1 规格） |

该话题直接输入给 `slam_toolbox`（建图/定位）和 `Nav2`（障碍物检测/代价地图）。

---

## 节点启动顺序与延迟

| 时间 | 节点 | 说明 |
|------|------|------|
| t=0s | `robot_state_publisher` | 发布 URDF 描述和静态 TF |
| t=0s | `ros2_control_node` | 加载硬件插件，打开 `/dev/ttyS7` |
| t=0s | `sllidar_node` | 连接 `/dev/ttyUSB0`，开始扫描 |
| t=0s | `twist_mux` | 等待速度指令输入 |
| t=0s | `odom_relay` | 桥接 `/diff_cont/odom` → `/odom` |
| t=2.0s | `spawner joint_broad` | 加载关节状态广播器 |
| t=2.5s | `spawner diff_cont` | 加载差速驱动控制器，开始发布里程计 |

> **延迟原因：** `joint_broad` 和 `diff_cont` 需要等待 `controller_manager` 服务就绪后才能注册，硬编码延迟是最简单可靠的等待方式。

---

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyS7` | STM32 串口（UART7-M2，硬件 UART） |
| `lidar_port` | `/dev/ttyUSB0` | 思岚 C1 雷达串口（USB 转串口） |
| `use_sim_time` | `false` | 使用系统时间（非仿真时间） |

示例：
```bash
ros2 launch my_bot_hw robot_bringup.launch.py serial_port:=/dev/ttyS7 lidar_port:=/dev/ttyUSB0
```
