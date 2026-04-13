# 包结构说明

本文档定义 `my_bot` 工作区内各 ROS2 包的职责边界，避免模型、硬件、SLAM、导航再次混在同一个包里。

## 包职责

### `my_bot`

负责通用机器人描述与仿真资源：

- `description/`
  - 机器人模型、link/joint、传感器安装位、TF 结构
- `worlds/`
  - Gazebo 世界文件
- `launch/launch_sim.launch.py`
  - 仿真启动入口
- `config/`
  - 仿真控制器、Gazebo bridge、通用 RViz 配置

规则：

- 机器人外形、尺寸、轮距、雷达位置、IMU 安装位，只改这里
- 不放实机串口、硬件驱动、SLAM 参数、Nav2 参数

### `my_bot_hw`

负责硬件驱动与实机基础 bringup：

- `src/`、`include/`
  - STM32 串口驱动、硬件接口实现
- `description/ros2_control_hw.xacro`
  - 实机 ros2_control 硬件接入
- `launch/robot_bringup.launch.py`
  - 实机底盘、激光、滤波、EKF、控制器启动
- `config/`
  - 实机控制器、EKF、IMU filter、laser filter 参数

规则：

- 只放硬件接口和实机底层启动
- 不定义机器人物理模型
- 不放 `slam_toolbox` 地图或 Nav2 参数

### `my_bot_slam`

负责 SLAM Toolbox 相关内容：

- `launch/slam.launch.py`
  - SLAM 独立入口
- `launch/nav_slam.launch.py`
  - 兼容的一键启动入口，同时拉起 SLAM 和 Nav2
- `config/mapper_params_*.yaml`
  - 仿真和实机的 SLAM 参数
- `config/maps/`
  - posegraph、保存地图等 SLAM 资产

规则：

- 地图文件统一放这里
- 建图和定位参数统一放这里

### `my_bot_nav`

负责 Nav2 相关内容：

- `launch/nav.launch.py`
  - Nav2 独立入口
- `launch/navigation_launch.py`
  - 导航节点编排
- `config/nav2_params_*.yaml`
  - 仿真和实机的 Nav2 参数

规则：

- Nav2 参数和导航 launch 统一放这里
- 不放硬件驱动或机器人模型

## 修改规则

### 改模型

例如：

- 车体尺寸
- 轮子位置
- 雷达安装位
- IMU frame

只修改 `my_bot/description/*`

### 改硬件

例如：

- 串口协议
- 驱动读写逻辑
- ros2_control hardware plugin
- 实机 EKF、激光滤波、控制器参数

只修改 `my_bot_hw/*`

### 改建图

例如：

- `slam_toolbox` 模式
- posegraph 路径
- 地图保存与加载

只修改 `my_bot_slam/*`

### 改导航

例如：

- costmap
- planner/controller 参数
- 行为树导航配置

只修改 `my_bot_nav/*`
