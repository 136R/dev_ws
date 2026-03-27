# my_bot_hw 频率配置总览

> 生成日期：2026-03-24
> 适用版本：ROS2 Humble，硬件底盘（非仿真）

---

## 1. 话题发布频率

| 话题                            | 频率            | 决定因素                                                                       | 配置来源                           |
| ------------------------------- | --------------- | ------------------------------------------------------------------------------ | ---------------------------------- |
| `/odom`                       | **50 Hz** | `controller_manager` update_rate 驱动 `diff_drive_controller` publish_rate | `config/hw_controllers.yaml`     |
| `/imu`                        | **50 Hz** | 每次 `read()` 都发布，与 controller_manager 同频                             | `src/stm32_serial_hw.cpp` read() |
| `/odometry/filtered`          | **50 Hz** | EKF frequency 参数                                                             | `config/ekf_hw.yaml`             |
| `/tf`（odom→base_footprint） | **50 Hz** | EKF publish_tf: true，随 EKF 频率输出                                          | `config/ekf_hw.yaml`             |
| `/map`（SLAM TF map→odom）   | **50 Hz** | transform_publish_period: 0.02 s = 50 Hz                                       | `config/mapper_params_hw*.yaml`  |
| `/cmd_vel`（Nav2 控制器输出） | **20 Hz** | controller_frequency                                                           | `config/nav2_params_hw.yaml`     |

---

## 2. 节点运行频率

| 节点                                    | 频率            | 参数名                               | 配置来源                           |
| --------------------------------------- | --------------- | ------------------------------------ | ---------------------------------- |
| `controller_manager`                  | **50 Hz** | `update_rate: 50`                  | `config/hw_controllers.yaml:3`   |
| `diff_drive_controller`（read/write） | **50 Hz** | 跟随 controller_manager              | `config/hw_controllers.yaml:3`   |
| `diff_drive_controller`（/odom 发布） | **50 Hz** | `publish_rate: 50.0`               | `config/hw_controllers.yaml:13`  |
| `ekf_filter_node`                     | **50 Hz** | `frequency: 50.0`                  | `config/ekf_hw.yaml:3`           |
| `slam_toolbox`（TF 发布）             | **50 Hz** | `transform_publish_period: 0.02`   | `config/mapper_params_hw*.yaml`  |
| Nav2 `controller_server`              | **20 Hz** | `controller_frequency: 20.0`       | `config/nav2_params_hw.yaml:87`  |
| Nav2 `planner_server`                 | **20 Hz** | `expected_planner_frequency: 20.0` | `config/nav2_params_hw.yaml:296` |
| Nav2 `smoother_server`                | **20 Hz** | `smoothing_frequency: 20.0`        | `config/nav2_params_hw.yaml:385` |
| Nav2 `behavior_server`                | **10 Hz** | `cycle_frequency: 10.0`            | `config/nav2_params_hw.yaml:337` |
| Nav2 `costmap_2d`（local）            | **10 Hz** | `update_frequency: 10.0`           | `config/nav2_params_hw.yaml:196` |
| Nav2 `costmap_2d`（local 发布）       | **2 Hz**  | `publish_frequency: 2.0`           | `config/nav2_params_hw.yaml:197` |
| Nav2 `costmap_2d`（global）           | **1 Hz**  | `update_frequency: 1.0`            | `config/nav2_params_hw.yaml:242` |
| Nav2 `costmap_2d`（global 发布）      | **1 Hz**  | `publish_frequency: 1.0`           | `config/nav2_params_hw.yaml:243` |
| Nav2 `waypoint_follower`              | **20 Hz** | `loop_rate: 20`                    | `config/nav2_params_hw.yaml:371` |

---

## 3. 代码内硬编码频率参数

| 参数 / 魔数                                          | 值                    | 含义                                                  | 代码位置                                  |
| ---------------------------------------------------- | --------------------- | ----------------------------------------------------- | ----------------------------------------- |
| `read_odom_frame(18)`                              | **18 ms 超时**  | 0x0A 响应等待上限，留 2 ms 余量（20 ms 周期）         | `src/stm32_serial_hw.cpp:166`           |
| `serial_read_timeout(1, std::min(deadline_ms, 5))` | **5 ms**        | 每次等待 0x5A 帧头的超时步长                          | `src/stm32_serial_hw.cpp:357`           |
| controller spawner `period=2.0`                    | **2.0 s**       | joint_state_broadcaster 启动延迟                      | `launch/robot_bringup.launch.py:72`     |
| controller spawner `period=2.5`                    | **2.5 s**       | diff_drive_controller 启动延迟（在 joint_broad 之后） | `launch/robot_bringup.launch.py:83`     |
| RPLIDAR `serial_baudrate: 460800`                  | **460800 baud** | 思岚 C1 激光雷达波特率（非 115200）                   | `launch/robot_bringup.launch.py:101`    |
| STM32 串口波特率                                     | **115200 baud** | cfsetispeed/cfsetospeed B115200                       | `src/stm32_serial_hw.cpp:open_serial()` |

---

## 4. EKF 传感器超时阈值

| 参数                               | 值               | 含义                                  | 配置来源                          |
| ---------------------------------- | ---------------- | ------------------------------------- | --------------------------------- |
| `sensor_timeout`                 | **0.08 s** | 传感器静默超过此时长 EKF 停止等待该源 | `config/ekf_hw.yaml:4`          |
| `cmd_vel_timeout`                | **0.5 s**  | diff_drive_controller 停止无指令超时  | `config/hw_controllers.yaml:36` |
| slam_toolbox `transform_timeout` | **0.5 s**  | TF 查询超时                           | `config/mapper_params_hw*.yaml` |

---

## 5. 频率链路图

```
STM32 串口（115200 baud）
  └─ read() 18ms 超时 ─→ controller_manager @ 50 Hz
        ├─ diff_drive_controller ─→ /odom @ 50 Hz
        │                          /tf (diff_cont, 已关闭 enable_odom_tf:false)
        ├─ stm32_serial_hw read() ─→ /imu @ 50 Hz
        │
        └─ ekf_filter_node @ 50 Hz
              ├─ 输入：/odom (vx)  + /imu (yaw)
              ├─ sensor_timeout: 0.08 s
              └─ 输出：/odometry/filtered @ 50 Hz
                       /tf (odom→base_footprint) @ 50 Hz

slam_toolbox
  ├─ 输入：/scan + /tf (odom→base_footprint)
  └─ 输出：/tf (map→odom) @ 50 Hz（transform_publish_period: 0.02）

Nav2 controller_server @ 20 Hz
  ├─ 输入：/odometry/filtered
  └─ 输出：/cmd_vel @ 20 Hz
```

---

## 6. 关键约束说明

| 约束                                    | 计算                                                  |
| --------------------------------------- | ----------------------------------------------------- |
| `sensor_timeout ≥ 2 × (1/imu_hz)`   | 0.08 s ≥ 2 × 20 ms = 0.04 s ✅（留 4 倍余量）       |
| `model_dt = 1 / controller_frequency` | 0.05 s = 1 / 20.0 ✅（Nav2 DWB 要求严格匹配）         |
| 串口时序余量（50 Hz 周期 20 ms）        | 18 ms 超时 + 2 ms 裕量：若超时则本周期跳过，不阻塞 ✅ |
| EKF 频率 ≥ 最快传感器频率              | 50 Hz = 50 Hz（/odom 与 /imu 均为 50 Hz）✅           |
