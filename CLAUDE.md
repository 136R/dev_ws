# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

平台信息

- **硬件：** Orange Pi 5 Pro（ARM Cortex-A76，RK3588），运行 ROS2 Humble
- **机器人：** 自制差速驱动小车，STM32F303RBT6 MCU 负责底层电机与传感器控制
- **固件位置：** `firmware/my_robot_stm32/`（STM32CubeIDE 工程，不参与 colcon 构建）

## 构建

```bash
cd ~/dev_ws
colcon build --symlink-install                              # 编译整个工作空间
colcon build --packages-select my_bot_hw --symlink-install # 编译单个包
source install/setup.bash                                  # 每次新终端必须执行
```

构建日志：`~/dev_ws/log/latest_build/<package>/stdout_stderr.log`

## 包架构

`src/` 下共五个包：

| 包名           | 职责                                                                    |
| -------------- | ----------------------------------------------------------------------- |
| `my_bot`       | 机器人 URDF/Xacro 描述 + Gazebo 仿真资源                                |
| `my_bot_hw`    | ros2_control`SystemInterface` 插件 —— 与 STM32 串口通信，实机 bringup |
| `my_bot_slam`  | SLAM Toolbox 的 launch 文件、参数配置、保存的地图                       |
| `my_bot_nav`   | Nav2 的 launch 文件和参数文件                                           |
| `sllidar_ros2` | 思岚 C1 激光雷达厂商驱动                                                |

`my_bot` 是仿真与实机共用的机器人描述包。xacro 参数 `sim_mode` 用于在 Gazebo 差速插件和 `my_bot_hw` ros2_control 硬件接口之间切换。

## 节点 / 话题架构（实机）

```
STM32F303 ←─UART 115200─→ Stm32SerialHardware (ros2_control 插件)
                                   │  全链路 100 Hz
              ┌────────────────────┼────────────────────┐
              ▼                    ▼                     ▼
  DiffDriveController     IMUSensorBroadcaster   JointStateBroadcaster
   /diff_cont/odom         /imu_broad/imu          /joint_states
       │                        │
       ▼ (relay)                │   ← EKF 直接吃原始 IMU
     /odom                      │     (ComplementaryFilter 未启用)
       │                        │
       └──────────┬─────────────┘
                  ▼
           EKF (robot_localization) @100Hz
           /odometry/filtered  +  TF odom→base_footprint
           （diff_cont 的 enable_odom_tf: false，TF 只有 EKF 发）

思岚 C1 ←─USB 460800─→ sllidar_node → /scan → LaserFilter → /scan_filtered

TwistMux: /cmd_vel_keyboard (90) + /cmd_vel_nav (70) → /diff_cont/cmd_vel_unstamped
```

详见 `src/my_bot_hw/docs/话题与频率.md`。

## 配置文件

| 文件                                                  | 用途                                   |
| ----------------------------------------------------- | -------------------------------------- |
| `src/my_bot_hw/config/hw_controllers.yaml`            | 控制器频率、轮系几何校正倍数、速度限制 |
| `src/my_bot_hw/config/ekf_hw.yaml`                    | EKF 融合权重与协方差                   |
| `src/my_bot_hw/config/laser_filters.yaml`             | 激光雷达距离/角度过滤                  |
| `src/my_bot_slam/config/mapper_params_{hw,sim}.yaml`  | SLAM Toolbox 调参                      |
| `src/my_bot_nav/config/{hw,sim}/nav2_params_*_{rpp,neupan}.yaml` | Nav2 调参（**只有这四个会被加载**） |
| `src/my_bot_nav/config/{hw,sim}/neupan_{hw,sim}.yaml` | NeuPAN 规划器调参                      |
| `firmware/my_robot_stm32/Core/Inc/app/robot_config.h` | STM32 PI 增益、编码器 CPR、减速比、机器人几何（**唯一真相源**） |

> MPPI / DWB / SmacPlanner 的旧参数与旧文档已删除（零引用、已不在技术栈内）。需要时去 git 历史找。

## 文档入口

**`docs/速查.md`** —— 跨包总速查 + 全局不变量表。**先看这个。**

各包（统一骨架：`README.md` 含不变量表 / `速查.md` / 专题）：

- `src/my_bot/docs/` —— 机器人描述 + Gazebo 仿真
- `src/my_bot_hw/docs/` —— 实机底盘、话题与频率、IMU 标定、激光过滤
- `src/my_bot_slam/docs/` —— 建图 / 定位 / 存图、建图调参
- `src/my_bot_nav/docs/` —— Nav2、NeuPAN、多地图与禁行区、调参
- `docs/APP/` —— 上位机 GUI（ROS_Flutter_Gui_App）集成：后端接口、前端补丁、复现部署
- `docs/workflows/实机同步与部署.md` —— 开发机 ↔ 开发板（**用 git，不要用 rsync**）、arm64 编译要求
- `docs/architecture/`、`docs/workflows/` —— 仓库级架构与工作流

> **动代码前先读对应包的 `docs/README.md` 的「不变量」表** —— 那里列的是"违反了就出错、
> 但从代码里看不出来"的约束。

