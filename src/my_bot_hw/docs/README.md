# my_bot_hw

实机底盘：`ros2_control` 的 `SystemInterface` 插件（与 STM32 串口通信）+ 实机 bringup。

## 一分钟全局

```
ros2 launch my_bot_hw robot_bringup.launch.py
```

起这些节点（含启动延迟）：

```
robot_state_publisher          ← URDF（my_bot 包的 xacro，sim_mode=false）
ros2_control_node              ← 加载 Stm32SerialHardware 插件
  ├─ joint_broad   (t+2.0s)    → /joint_states
  ├─ imu_broad     (t+2.5s)    → /imu_broad/imu
  └─ diff_cont     (t+3.0s)    → /diff_cont/odom
sllidar_node                   → /scan           （思岚 C1）
laser_filter                   → /scan_filtered
twist_mux                      → /diff_cont/cmd_vel_unstamped
odom_relay                     /diff_cont/odom → /odom
ekf_filter_node  (t+4.0s)      /odom + /imu_broad/imu → /odometry/filtered + TF odom→base_footprint
```

```
STM32F303 ←─ UART /dev/ttyS7 @115200 ─→ Stm32SerialHardware
思岚 C1   ←─ USB /dev/ttyUSB0 @460800 ─→ sllidar_node
```

---

## ⚠️ 不变量

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **EKF 融合的是 `/imu_broad/imu`，不是 `/imu/data`** | 去接 `imu_complementary_filter` 会发现没人订阅 —— 那行是**注释掉的** | `config/ekf_hw.yaml:28`（`imu0: /imu_broad/imu`，上一行 `# imu0: /imu/data` 已注释） |
| **`diff_cont` 不发 TF，TF 由 EKF 发** | 两处都发 → `odom→base_footprint` 打架 | `hw_controllers.yaml:39` `enable_odom_tf: false`；`ekf_hw.yaml:11` `publish_tf: true` |
| **全链路 100 Hz**（STM32 控制环 / controller_manager / EKF） | 改其中一个而不改其余 → 时序错配 | `robot_config.h` `CONTROL_FREQ_HZ 100`；`hw_controllers.yaml` `update_rate: 100`；`ekf_hw.yaml` `frequency: 100.0` |
| **`sensor_timeout` 必须 ≥ 2×(1/传感器频率)** | EKF 因单帧丢失就丢弃该数据源 | `ekf_hw.yaml` `sensor_timeout: 0.08` ≥ 2×10ms ✓（留 4 倍余量） |
| **雷达波特率是 460800，不是 115200** | `/scan` 无数据 | 思岚 C1 专用；`robot_bringup.launch.py` |
| **`straight_drive_corrector` 目前是关闭的** | 别以为它在生效 | `robot_bringup.launch.py` 里整段被注释 |
| **控制器有启动延迟（2.0/2.5/3.0/4.0s）** | 起完 launch 立刻发指令会丢 | `TimerAction` 分级延迟，EKF 等 imu_broad 就绪 |
| **编译必须带 `--cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON`** | neupan 链接期报 `relocation R_AARCH64_*`，编不过（arm64 特有，x86_64 上侥幸能过） | `libneupan.a` 要链进 controller 插件的动态库 |
| **`src/gz_ros2_control` 要加 `COLCON_IGNORE`** | 板子没装 Gazebo，它找不到 `ignition-gazebo6` 而失败，**连带整个 build 中止** | 仿真专用包，实机用不上 |

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 启动、遥控、话题调试命令 | [速查.md](速查.md) |
| 话题怎么流、各环节频率是多少 | [话题与频率.md](话题与频率.md) |
| **换了轮胎/底盘/雷达，要重新标定** | [里程计与雷达标定.md](里程计与雷达标定.md) |
| IMU 加速度计标定 | [imu标定.md](imu标定.md) |
| 激光雷达过滤链调参 | [激光过滤调参.md](激光过滤调参.md) |
| STM32 侧（PI 增益、编码器、减速比） | `firmware/my_robot_stm32/Core/Inc/app/robot_config.h` |
| 开发机 ↔ 开发板怎么同步、怎么从零部署 | [docs/workflows/实机同步与部署.md](../../../docs/workflows/实机同步与部署.md) |
