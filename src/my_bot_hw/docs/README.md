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
  ├─ joint_broad   (延迟①)     → /joint_states
  ├─ imu_broad     (延迟②)     → /imu_broad/imu
  └─ diff_cont     (延迟③)     → /diff_cont/odom
sllidar_node                   → /scan           （思岚 C1）
laser_filter                   → /scan_filtered
twist_mux                      → /diff_cont/cmd_vel_unstamped
odom_relay                     /diff_cont/odom → /odom
ekf_filter_node  (延迟④)       /odom + /imu_broad/imu → /odometry/filtered + TF odom→base_footprint
```

四级延迟的秒数见 `robot_bringup.launch.py` 的 `TimerAction`（顺序不能乱，EKF 必须等 imu_broad 就绪）。

```
STM32F303 ←─ UART /dev/ttyS7 ─→ Stm32SerialHardware
思岚 C1   ←─ USB /dev/ttyUSB0 ─→ sllidar_node
```

两条链路的波特率不同，都在 `robot_bringup.launch.py` 里。

---

## ⚠️ 不变量

| 不变量 | 违反后的症状 | 去哪看 |
| --- | --- | --- |
| **EKF 融合的是 `/imu_broad/imu`，不是 `/imu/data`** | 去接 `imu_complementary_filter` 会发现没人订阅 —— 那行是**注释掉的** | `ekf_hw.yaml` 的 `imu0`（它上一行的 `# imu0: /imu/data` 是注释） |
| **`diff_cont` 不发 TF，TF 由 EKF 发** | 两处都发 → `odom→base_footprint` 打架 | `hw_controllers.yaml` 的 `enable_odom_tf`；`ekf_hw.yaml` 的 `publish_tf` |
| **全链路同频**（STM32 控制环 / controller_manager / EKF） | 改其中一个而不改其余 → 时序错配 | 三处必须相等：`robot_config.h` 的 `CONTROL_FREQ_HZ`、`hw_controllers.yaml` 的 `update_rate`、`ekf_hw.yaml` 的 `frequency` |
| **`sensor_timeout` 必须 ≥ 2×(1/传感器频率)** | EKF 因单帧丢失就丢弃该数据源 | `ekf_hw.yaml` 的 `sensor_timeout`，对照上一行的传感器频率算 |
| **雷达波特率与底盘串口不同，别照抄** | `/scan` 无数据 | `robot_bringup.launch.py` 的 `serial_baudrate`（思岚 C1 专用值） |
| **`straight_drive_corrector` 目前是关闭的** | 别以为它在生效 | `robot_bringup.launch.py` 里整段被注释 |
| **控制器分四级延迟启动** | 起完 launch 立刻发指令会丢 | `robot_bringup.launch.py` 的 `TimerAction`，EKF 必须等 imu_broad 就绪 |
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
