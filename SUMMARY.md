# SUMMARY：按子系统深入

> 定位：按子系统深入项目内部，写决策理由 / 原理 / 调参复盘 / 仿真 vs 实机差异 / 当前未理解的点。
> 何时读：当 [TIMELINE.md](TIMELINE.md) 的"做了什么"已经不够，你想知道"为什么这么做、内部是怎么回事"。
> 下位机相关：一律链向 [STM32\_SUMMARY.md](STM32_SUMMARY.md)，本篇不重复展开。

---

## S0. 三档文档关系


| 文档                                                                                     | 何时翻                                             |
| ---------------------------------------------------------------------------------------- | -------------------------------------------------- |
| [TIMELINE.md](TIMELINE.md)                                                               | 想"重做一遍"或回忆"我那时是怎么走过来的"           |
| SUMMARY.md（本篇）                                                                       | 想理解"为什么这么做 / 原理是什么 / 调参思路是什么" |
| [STM32\_SUMMARY.md](STM32_SUMMARY.md)                                                    | 下位机固件细节、协议、PI 调试                      |
| [docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)                                   | Nav2 调参权威源                                    |
| [docs/上位机从零配置/香橙派5Pro 从零配置.md](docs/上位机从零配置/香橙派5Pro 从零配置.md) | 环境从零配置                                       |
| src/<pkg>/docs/\*\_cheatsheet.md                                                         | 调试命令速查                                       |

> 阅读说明：本文档中 ROS2 内部实现细节（ros2\_control 调度、Nav2 行为树、EKF 内部数学等）由 AI 根据源码与配置整理，未经本人深入验证。决策与调参经验是本人亲历，实现细节请以源码为准。

---

## S1. 系统全景

### 硬件 / 软件栈一行

* 硬件：Orange Pi 5 Pro（RK3588，Cortex-A76）+ STM32F303RBT6 + ICM-42688-P 6 轴 IMU + 思岚 C1 激光雷达 + 自制差速底盘
* 软件：Ubuntu 22.04 + ROS2 Humble + ros2\_control（diff\_drive + imu\_sensor\_broadcaster）+ robot\_localization（EKF）+ slam\_toolbox + Nav2 + twist\_mux + laser\_filters

### 数据流图（已根据 IMU 实际处理位置修订）

> CLAUDE.md 中的旧图保留了 ROS2 端的 ComplementaryFilter，那是早期方案。
> 现状：IMU 解算在 STM32 完成（Fusion 库 Madgwick），ROS2 端不再做姿态融合。

```
  STM32F303 (Madgwick yaw, ICM-42688-P)
        │
        │  UART 115200  (协议见 STM32_SUMMARY §M2)
        │  上行 0x02 FEEDBACK @ 50 Hz
        │  下行 0x01 VEL_CMD  @ 100 Hz
        ▼
  Stm32SerialHardware  (ros2_control SystemInterface 插件, my_bot_hw)
        │
        ├──────────────┬─────────────────────┐
        ▼              ▼                      ▼
  DiffDriveController  IMUSensorBroadcaster  JointStateBroadcaster
    /diff_cont/odom     /imu_broad/imu        /joint_states
        │                   │
        ▼ topic_tools/relay │
       /odom                │
        │ vx only            │ yaw only
        └─────────┬──────────┘
                  ▼
      ekf_filter_node (robot_localization)
        /odometry/filtered  +  TF odom → base_footprint

  ───────────────────────── 雷达 ─────────────────────────
  思岚 C1  ──USB 460800──→  sllidar_node  →  /scan
                                              │
                                              ▼  laser_filters (range + speckle + angular)
                                            /scan_filtered  →  SLAM / Nav2

  ───────────────────────── 控制流 ─────────────────────────
  /cmd_vel_keyboard (优先级 90)
                          │
  /cmd_vel_nav      (优先级 70)
                          │
                          ▼  twist_mux
                  /diff_cont/cmd_vel_unstamped  →  DiffDriveController
```

与 CLAUDE.md 旧图的差异：

1. 去掉了 ROS2 端的 ComplementaryFilter 节点 —— 现在不存在
2. EKF 的 imu0 是 /imu\_broad/imu（不是 /imu/data，详见 [ekf\_hw.yaml:28](src/my_bot_hw/config/ekf_hw.yaml#L28)）
3. /imu\_broad/imu 里的 yaw 是 STM32 Madgwick 解算结果，不是原始 IMU

### 5 个包职责简表


| 包            | 职责                                                                    | 速查表                                                                |
| ------------- | ----------------------------------------------------------------------- | --------------------------------------------------------------------- |
| my\_bot       | URDF/xacro + 仿真资源（Gazebo 世界、控制器、bridge）                    | [sim\_cheatsheet.md](src/my_bot/docs/速查.md)               |
| my\_bot\_hw   | ros2\_controlSystemInterface 插件 + 实机 bringup launch + 滤波/EKF 配置 | [robot\_hw\_cheatsheet.md](src/my_bot_hw/docs/速查.md) |
| my\_bot\_slam | SLAM Toolbox launch + 三套 mapper\_params + 已保存地图                  | [slam\_cheatsheet.md](src/my_bot_slam/docs/速查.md)        |
| my\_bot\_nav  | Nav2 launch + 五套 nav2\_params（含 BT 配置）                           | [nav\_cheatsheet.md](src/my_bot_nav/docs/速查.md)           |
| sllidar\_ros2 | 思岚 C1 厂商驱动（USB 460800）                                          | 不调，开箱即用                                                        |

---

## S2. 仿真章节（独立一节）

### 仿真期的价值

先仿后实让以下事情可以在没有真车的桌面上调通：

* URDF 装配错误（mesh 错位、关节方向错、collision 体过大/过小）
* TF 链完整性（base\_footprint → base\_link → wheel\_\*、激光与 IMU 的外参占位）
* ros2\_control 控制器加载顺序与名字匹配
* diff\_drive 输出的 odom 量纲（线/角速度方向）
* costmap inflation 数值的视觉感
* planner / controller 的行为感

### 关键文件


| 文件                                                                                    | 用途                                                       |
| --------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| [src/my\_bot/launch/launch\_sim.launch.py](src/my_bot/launch/launch_sim.launch.py)      | 一键拉 Gazebo + spawn + 控制器 + bridge                    |
| [src/my\_bot/launch/start\_world.launch.py](src/my_bot/launch/start_world.launch.py)    | 单独启动世界                                               |
| [src/my\_bot/launch/spawn\_robot.launch.py](src/my_bot/launch/spawn_robot.launch.py)    | 单独 spawn 机器人                                          |
| [src/my\_bot/config/ekf\_sim.yaml](src/my_bot/config/ekf_sim.yaml)                      | 仿真侧 EKF（与实机版不同，见 §S5）                        |
| [src/my\_bot/config/my\_controllers.yaml](src/my_bot/config/my_controllers.yaml)        | 仿真侧 diff\_drive 控制器（实机版是 hw\_controllers.yaml） |
| [src/my\_bot/config/gz\_bridge.yaml](src/my_bot/config/gz_bridge.yaml)                  | Gazebo↔ROS2 topic 桥                                      |
| [src/my\_bot/config/laser\_filters\_sim.yaml](src/my_bot/config/laser_filters_sim.yaml) | 仿真侧滤波（与实机版结构相同，阈值可能不同）               |

### sim\_mode xacro 切换机制

robot.urdf.xacro 顶层接受参数 sim\_mode:=true|false：

* sim\_mode:=true（默认）：加载 Gazebo 差速插件 + Gazebo IMU + 仿真激光
* sim\_mode:=false：加载 ros2\_control\_hw.xacro（实机硬件接口）

同一份 URDF 通过这个开关复用于仿真和实机。实机 bringup 在 [robot\_bringup.launch.py:36-42](src/my_bot_hw/launch/robot_bringup.launch.py#L36-L42) 里显式传 sim\_mode:=false。

### Gazebo↔ROS2 桥（部分要点）

* 见 gz\_bridge.yaml 列出的 topic 映射

### 仿真期跑 Nav2 vs 实机的差别


| 维度       | 仿真                                | 实机                                                |
| ---------- | ----------------------------------- | --------------------------------------------------- |
| IMU        | Gazebo 零漂 IMU，融合全 IMU 字段    | STM32 Madgwick yaw，EKF 只融 yaw                    |
| 雷达       | Gazebo 仿真激光，直接发/scan        | 思岚 C1 USB，经 laser\_filters 发/scan\_filtered    |
| 控制器配置 | my\_controllers.yaml（轮径 0.035m） | hw\_controllers.yaml（轮径 0.033401m + multiplier） |
| EKF 配置   | ekf\_sim.yaml                       | ekf\_hw.yaml（odom0 只用 vx；imu0 只用 yaw）        |
| Nav2 配置  | nav2\_params\_sim.yaml              | nav2\_params\_hw.yaml（ThetaStar + RPP）            |
| time       | 用use\_sim\_time:=true              | 真实时间                                            |

### 仿真验收清单（最小必过项）

1. ros2 launch my\_bot launch\_sim.launch.py 没报错，Gazebo 起来
2. ros2 topic list 看到 /odom、/imu、/scan、/joint\_states、/clock
3. ros2 run tf2\_tools view\_frames 生成的 TF 树有 map（建图后） / odom / base\_footprint / base\_link / wheel\_\* / imu\_link / laser\_frame
4. teleop 能让机器人在 Gazebo 里前进、原地转
5. ros2 launch my\_bot\_slam slam.launch.py use\_sim\_time:=true mode:=mapping 配合遥控能在 RViz 看到地图增长

### 仿真期已经踩过的坑

1. 跟着 articulatedrobotics 的教程一般到 slam toolbox 前都不会有大问题，到 slam toolbox 阶段会有以下问题
   1. ros2 topic echo /scan 订阅雷达数据或者频率会卡死，看 WSL 通信是不是镜像模式（在 C:\\Users\\<用户名>\\.wslconfig）。是的话配置 CycloneDDS 不用默认的 FastDDS 与 ROS2 Daemon 管理
   2. humble 的带参运行 API 是 slam\_params\_file，教程是 params\_file，这会导致参数加载失败
2. nav2 默认用的是 DWB 控制器，对于通道大于 50 cm 的场景够用，小于就换 RPP（通过全局进行避障）

---

## S3. 主题深入（按子系统，实机为主）

每个子系统按"核心职责 / 关键文件 / 复现要点 / 决策理由 / 链 cheatsheet"展开。

### S3.1 my\_bot —— 统一 URDF 与 sim\_mode 切换

核心职责：成为仿真和实机唯一的机器人描述源。

关键文件

* [src/my\_bot/description/robot.urdf.xacro](src/my_bot/description/robot.urdf.xacro)（主入口，接受 sim\_mode、ros2\_control\_config、serial\_port、lidar\_yaw 等参数）
* [src/my\_bot/description/ros2\_control.xacro](src/my_bot/description/ros2_control.xacro)（仿真侧硬件接口 xacro）
* [src/my\_bot\_hw/description/ros2\_control\_hw.xacro](src/my_bot_hw/description/ros2_control_hw.xacro)（实机侧硬件接口 xacro，由 robot\_bringup launch 注入）

复现要点

* 仿真：直接 launch\_sim.launch.py
* 实机：robot\_bringup.launch.py 内部 xacro ... sim\_mode:=false ros2\_control\_config:=ros2\_control\_hw.xacro serial\_port:=... lidar\_yaw:=3.115414361

决策理由

* 用 xacro 而非分两份 URDF：避免双源维护，几何参数一处改两处生效
* 把 sim/hw 的 ros2\_control 描述拆成两个 xacro，由 launch 选择 —— 比 `xacro:if` 更清晰

链向：[sim\_cheatsheet.md](src/my_bot/docs/速查.md)

### S3.2 my\_bot\_hw —— ros2\_control 硬件接口 + 串口 + EKF

核心职责

* 提供 Stm32SerialHardware：ros2\_control 的 hardware\_interface::SystemInterface 实现，负责串口读写 + 把 STM32 数据暴露成 hardware command/state interfaces
* 编排实机 bringup launch（一条命令拉起所有实机节点）
* 持有 EKF、laser\_filter、twist\_mux、odom relay 等的实机配置

关键文件

* [launch/robot\_bringup.launch.py](src/my_bot_hw/launch/robot_bringup.launch.py)
* [config/hw\_controllers.yaml](src/my_bot_hw/config/hw_controllers.yaml)：diff\_cont + joint\_broad + imu\_broad
* [config/ekf\_hw.yaml](src/my_bot_hw/config/ekf_hw.yaml)：odom0 + imu0 配置
* [config/laser\_filters.yaml](src/my_bot_hw/config/laser_filters.yaml)：range + speckle + angular\_bounds 三道
* [config/imu\_complementary\_filter.yaml](src/my_bot_hw/config/imu_complementary_filter.yaml)：当前 launch 未使用（IMU 处理迁去 STM32 后的遗留文件）

复现要点

* 串口分配：STM32 在 /dev/ttyS7（UART7-M2），LiDAR 在 /dev/ttyUSB0（460800）
* 控制器启动顺序：joint\_broad（2.0s）→ imu\_broad（2.3s）→ diff\_cont（2.6s）→ EKF（4.0s）—— 用 TimerAction 等 controller\_manager 就绪
* enable\_odom\_tf: false 在 diff\_cont 端：odom→base\_footprint TF 唯一由 EKF 发布

决策理由

* IMU 不在 ROS2 端处理（详见 §S4.4）：launch 里看不到 imu\_complementary\_filter 节点；imu\_complementary\_filter.yaml 仅保留作历史档案。EKF 直接订阅 /imu\_broad/imu，里面的 yaw 来自 STM32。
* EKF 只用 vx + yaw：避免编码器 vyaw 和 IMU yaw 冲突，参考 [ekf\_hw.yaml:14-22](src/my_bot_hw/config/ekf_hw.yaml#L14-L22) 注释
* 几何 multiplier：
  * wheel\_separation\_multiplier = 0.98012 —— 标定后实测有效轮距 ≈ 0.1715 m
  * right\_wheel\_radius\_multiplier = 0.99 —— 补偿右轮制造/磨损偏差
  * 标定方法（更多细节见 §S4.6）：
    1. 走直线看往哪边偏移，如：向右偏移，就把 right\_wheel\_radius\_multiplier 调小
    2. 看走 1 米 /odom 显示是否 1 m，若实际 0.98 则调整 wheel\_separation\_multiplier
    3. 公式：目标 / 实际
* base\_frame\_id: base\_footprint：所有控制器和 EKF 一致用这个，避免 base\_link 高度引入额外的 z 偏置
* publish\_rate: 100.0、velocity\_rolling\_window\_size: 5：5 个 10 ms 窗口 ≈ 50 ms 平滑

与 STM32 的接口：见 [STM32\_SUMMARY.md §M2](STM32_SUMMARY.md#m2-串口通信协议与-ros2-的数据格式)

链向：[robot\_hw\_cheatsheet.md](src/my_bot_hw/docs/速查.md)（含调试命令、TF 链排查、八条诊断清单）

### S3.3 sllidar\_ros2 + laser\_filter —— 雷达链路

核心职责

* sllidar\_ros2：思岚厂商驱动，发布 /scan（USB 460800）
* laser\_filters/scan\_to\_scan\_filter\_chain：三道滤波 → 发布 /scan\_filtered

滤波链（见[laser\_filters.yaml](src/my_bot_hw/config/laser_filters.yaml)）

1. range\_filter：盲区点（< 0.1m） 与超量程点（> 4.0m）替换为 ±inf
2. speckle\_filter：与窗口内点距离差 > 0.6m 视为斑点删除（消除玻璃反射、粉尘）
3. angular\_bounds\_filter：占位状态（当前 lower=upper=0.0，未启用），用于屏蔽固定遮挡扇区

复现要点

* C1 必须 460800 波特率（不是 115200，见 [robot\_bringup.launch.py:113](src/my_bot_hw/launch/robot_bringup.launch.py#L113)）
* frame\_id: laser\_frame，URDF 的 lidar joint 给 lidar\_yaw:=3.115414361（实测标定值，几乎是 π 但不等）
* SLAM 与 Nav2 都订阅 /scan\_filtered，不订 /scan

决策理由

* frame\_id: laser\_frame + lidar\_yaw 精确标定：避免地图随车朝向偏转
* 三道滤波串联：单独的 range 不够，speckle 必须配合
* angular\_bounds 占位：暂未发现固定遮挡，但保留位置便于将来增加

链向：滤波细节见 laser\_filters.yaml 注释；C1 调试见 [robot\_hw\_cheatsheet.md](src/my_bot_hw/docs/速查.md)

### S3.4 my\_bot\_slam —— 三套 mapper 配置的用途差异

核心职责：SLAM Toolbox 的 launch 和参数。建图（mapping） / 定位（localization） / 仿真各一套。

三套配置对照


| 文件                             | mode                                 | scan\_topic     | 用途                                   |
| -------------------------------- | ------------------------------------ | --------------- | -------------------------------------- |
| mapper\_params\_hw.yaml          | mapping（注释里准备切 localization） | /scan\_filtered | 实机日常（launch 通过mode:= 参数切换） |
| mapper\_params\_hw\_mapping.yaml | mapping                              | /scan\_filtered | 实机建图专用（参数偏 mapping）         |
| mapper\_params\_sim.yaml         | localization                         | /scan           | 仿真用（包含已保存地图路径）           |

launch 参数：

* [slam.launch.py](src/my_bot_slam/launch/slam.launch.py) 接受 mode:=mapping|localization 参数选配置文件
* [nav\_slam.launch.py](src/my_bot_slam/launch/nav_slam.launch.py) 把 SLAM + Nav2 一起拉起

决策理由

* 实机订阅 /scan\_filtered（滤过的雷达数据） vs 仿真订阅 /scan（Gazebo 模拟无噪点）
* 建图模式与定位模式拆两个文件，避免每次切换都改大段参数

调参缺口：见 §S4.3

链向：[slam\_cheatsheet.md](src/my_bot_slam/docs/速查.md)

### S3.5 my\_bot\_nav —— TwistMux 优先级 + planner/controller 矩阵

核心职责：Nav2 launch + 五套备选参数 + BT 配置。

TwistMux 优先级（[robot\_bringup.launch.py:134-145](src/my_bot_hw/launch/robot_bringup.launch.py#L134-L145)）

* /cmd\_vel\_keyboard 优先级 90（teleop 最高，应急可手动覆盖 Nav2）
* /cmd\_vel\_nav 优先级 70（Nav2 输出）
* 输出 → /diff\_cont/cmd\_vel\_unstamped

Planner / Controller 矩阵（五套配置文件）


| 配置                    | Planner       | Controller                 | 现状                      |
| ----------------------- | ------------- | -------------------------- | ------------------------- |
| nav2\_params\_hw.yaml   | ThetaStar     | RegulatedPurePursuit (RPP) | 当前实机方案              |
| nav2\_params\_mppi.yaml | SmacPlanner2D | MPPI                       | 试用未采用（详见 §S4.2） |
| nav2\_params\_rpp.yaml  | ThetaStar     | RPP                        | RPP 调试中间版            |
| nav2\_params\_dwb.yaml  | ThetaStar     | DWB                        | DWB 试用                  |
| nav2\_params\_sim.yaml  | （仿真版）    | —                         | 仿真期用                  |

BT 改动：用 IsPathValid 替代周期性重规划（详见 [调参素材 §RPP 第 5 条](docs/调参素材/调参素材.md)），解决"同位置不动路径还反复变"。

决策理由：完整决策链见 §S4.1、§S4.2 与 [调参素材](docs/调参素材/调参素材.md)。

链向：[nav\_cheatsheet.md](src/my_bot_nav/docs/速查.md)、[docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)

---

## S4. 调参复盘（与调参素材联动）

本章是索引 + 节选 + 引导问题，权威细节在 [docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)。

### S4.1 Nav2 调参（ThetaStar + RPP）

调参素材已写好的部分：[调参素材.md §nav2](docs/调参素材/调参素材.md#nav2)

调参顺序（直接引调参素材的结论）：

1. Costmap：先把 footprint 与膨胀做对（inflation\_radius、cost\_scaling\_factor）
2. global\_costmap → ThetaStar：w\_traversal\_cost（远离障碍） vs w\_euc\_cost（最短路径）
3. local\_costmap → RPP：use\_velocity\_scaled\_lookahead\_dist: true，lookahead\_dist: 0.3 等
4. BT：用 IsPathValid 替代周期性重规划

实机 footprint（[调参素材](docs/调参素材/调参素材.md#footprint-验证必调前置)）：

* [[0.05, 0.126], [0.05, -0.126], [-0.104, -0.126], [-0.104, 0.126]]
* 长 0.154 m × 宽 0.252 m（含安全余量）

窄通道关键（RPP 第 2-6 条）：

* 关 use\_collision\_detection（避免前馈误判碰撞触发急停）
* 开 use\_rotate\_to\_heading（开启需关 allow\_reversing）
* 别配 Collision Monitor

### S4.2 MPPI / DWB 试用结论

直接引 [调参素材 §local\_costmap](docs/调参素材/调参素材.md#local_costmap局部规划器) 的现象：

DWB：

1. /local\_plan 不跟随全局 /plan —— 全局绕开墙，DWB 为最短贴墙走
2. 各种转角规划失败

MPPI：

1. 用默认参数 / 调参后出现异常行为（接近目标时规划"倒车入库"），未能定位原因
2. 直线目标却歪着走
3. (0,0) → (1, 0.5) 时先走 X 然后卡死

结论：不采用，回归 ThetaStar + RPP。

### S4.3 SLAM Toolbox 调参

前置：SLAM 前要做 odom 校准 —— RViz2 看 /odom TF 选择 odom，跑一圈看能不能回到原点。

调参细节：详见 [src/my\_bot\_slam/docs/slam\_mapping\_tuning\_guide.md](src/my_bot_slam/docs/建图调参.md)（关键参数、建图失败模式排查均在该文档）。

已确认要点：

* resolution：地图分辨率 0.05 即可（导航目标精度 0.05），更细只是耗 CPU
* 实机 SLAM 订阅 /scan\_filtered，仿真订阅 /scan —— 仿真雷达基本没有误差，不需要滤波
* num\_threads: 4：Orange Pi 5 Pro 上不需要开

### S4.4 IMU 处理位置迁移（ROS2 → STM32）

事实：

* 早期方案：ROS2 端做 imu\_calib 六面标定 + bias 估计 + 死区，仍有动态漂移
* 现行方案：STM32 用 Fusion 库的 Madgwick 算法，360° 误差约 0.5°
* imu\_complementary\_filter.yaml 在 my\_bot\_hw/config/ 下仍存在但 launch 未引用

经验记录：

* ROS2 端 imu\_calib 时的漂移表现：转 360° 实际可能是 361° / 359° / 358°，不稳定
* 死区值：陀螺低于 0.01–0.015 rad/s 时归零
* 切到 STM32 的判定：ROS2 测试没照（不理想），就到 STM32 层试了下，发现效果不错就一直留在 STM32 层
* 现行 0.5° / 360° 误差的测法：手动转 360° 看 yaw 累计
* 是否需要复位：会有累计误差，一直跑的情况下约 10 分钟复位一次（注：ICM-42688-P 6 轴无磁，无绝对航向参考）

### S4.5 EKF 协方差调参

事实（[ekf\_hw.yaml](src/my_bot_hw/config/ekf_hw.yaml)）：

* odom0\_config 只勾 vx（位置 6 个 false，其他全 false）
* imu0\_config 只勾 yaw（位置 5 个 false，yaw=true，其余 false）
* imu0\_differential: true（差分模式，用 yaw 增量而非绝对）
* 过程噪声协方差对角线已显式给出（process\_noise\_covariance 15×15 矩阵）

**经验记录**：

* **process_noise_covariance 对角线值**（x=0.05 / yaw=0.06 / vx=0.025 / vyaw=0.02）：沿用默认值，没有单独调过，也没有专门观察过 `/odometry/filtered` 的 yaw 漂移或 vyaw 抖动
* **`imu0_differential: true`**：用差分模式是为了能重置 EKF 发布的里程计
* **`twist_covariance_diagonal: [1.0e-6, 1e6, 1e6, 1e6, 1e6, 1.0e-3]`**：vx 给小协方差（1e-6，强信任），vyaw 给大协方差（1e-3，弱信任）—— 因为编码器估算的 yaw rate 不准，让 EKF 不从编码器拿 yaw rate
* **实机相比仿真 EKF 字段收窄**（仅 yaw）：实机 2D 导航只用得到 yaw；仿真 IMU 零漂所以可以全用，没有渐进收窄过程，是设计实机配置时直接砍掉的

### S4.6 轮系几何标定

事实（[hw\_controllers.yaml:25-34](src/my_bot_hw/config/hw_controllers.yaml#L25-L34)）：

* wheel\_separation: 0.175（注释说 needs measurement confirmation）
* wheel\_radius: 0.033401095（≈ 32.5 mm）
* wheel\_separation\_multiplier: 0.98012
* right\_wheel\_radius\_multiplier: 0.99
* left\_wheel\_radius\_multiplier: 1.0

对比 STM32 robot\_config.h：

* WHEEL\_RADIUS = 0.0325 m（与上位机 0.0334 略不同 —— STM32 这边没乘 multiplier）
* WHEEL\_TREAD = 0.171 m（与上位机 0.175 × 0.98012 ≈ 0.1715 几乎吻合）

**标定方法**：

* **`wheel_separation`**：卡尺量轮宽 + 两轮中心距得到
* **`wheel_radius` / multiplier**：通过走直线 + 走 1 米对照 `/odom` 显示值反推（详见 [§S3.2 标定方法](#s32-my_bot_hw--ros2_control-硬件接口--串口--ekf) 给出的两步流程）

  1. 走直线看偏哪边 —— 偏右就把 `right_wheel_radius_multiplier` 调小
  2. 走 1 m，看 `/odom` 显示数 —— 实际显示 0.98 m 就调整 `wheel_separation_multiplier`，公式：目标 / 实际
* 注释里的旧值 `# 1.022` / `# 0.99125` 是早期试过的版本，最终换成现在的 `0.98012` / `0.99` 是上述流程迭代收敛后的结果

### S4.7 PI 调试

PI 在 STM32 上调，不在 ROS2。详见 [STM32\_SUMMARY.md §M4](STM32_SUMMARY.md#m4-pi-调试流程)。

现行值（[robot\_config.h:54-57](firmware/my_robot_stm32/Core/Inc/app/robot_config.h#L54-L57)）：

* PI\_KP\_LEFT = 0.54   PI\_KI\_LEFT = 0.44
* PI\_KP\_RIGHT = 0.56  PI\_KI\_RIGHT = 0.44

---

## S5. 仿真 vs 实机差异表


| 配置项                                     | 仿真值                                   | 实机值                         | 原因                                           |
| ------------------------------------------ | ---------------------------------------- | ------------------------------ | ---------------------------------------------- |
| EKF odom0\_config vx                       | true                                     | true                           | 都用编码器线速度                               |
| EKF imu0\_config                           | yaw + roll + pitch + 全部 vel + 全部 acc | 仅 yaw                         | 仿真 IMU 零漂可全用；实机只信 STM32 解算的 yaw |
| EKF imu0\_differential                     | true                                     | true                           | 一致                                           |
| EKF process\_noise vx                      | 0.005                                    | 0.025                          | 沿用默认值，未单独调优（见 §S4.5）            |
| EKF process\_noise yaw                     | 0.02                                     | 0.06                           | 沿用默认值，未单独调优（见 §S4.5）            |
| diff\_cont wheel\_radius                   | 0.035                                    | 0.033401095                    | 真车实测尺寸不同                               |
| diff\_cont wheel\_separation               | 0.175                                    | 0.175（× 0.98012 multiplier） | 仿真不需要 multiplier 校正                     |
| diff\_cont velocity\_rolling\_window\_size | 3                                        | 5                              | 实机噪声更大，需要更长平滑窗                   |
| SLAM scan\_topic                           | /scan                                    | /scan\_filtered                | 实机激光有噪点，需 laser\_filters              |
| SLAM mode（默认）                          | localization（带预存地图）               | mapping（每次现场建/切定位）   | 仿真世界固定，实机经常换场地                   |
| laser\_filters                             | 同款三道过滤                             | 同款三道过滤                   | 一致；阈值上目前都是 0.1 / 4.0 m               |
| Nav2 planner / controller                  | （仿真版）                               | ThetaStar + RPP                | 实机经过多轮试错（见 §S4.2）                  |

> 完整字段对比可用 `diff -u src/my_bot/config/ekf_sim.yaml src/my_bot_hw/config/ekf_hw.yaml` 之类命令辅助。

---

## S6. 已知坑 / 待理解 / 学习路线

### S6.1 踩过且已解决的坑

**ROS2 / 实机侧**

* **雷达外参**：`lidar_yaw` 没设导致建图镜像翻转
* **Nav2 急停：MPPI 接近目标时倒车** —— 现象：接近目标点时规划出倒车路径，原因未定位，解决方式：放弃 MPPI，改用 RPP
* **Nav2 急停**：RPP 在窄通道触发 Collision Monitor
* **IMU**：在 ROS2 端 imu_calib 后仍漂，迁到 STM32 Madgwick
* **轮系**：右轮偏小走直线偏左 → `right_wheel_radius_multiplier = 0.99`

**STM32 侧**

1. 速度环反馈要用编码增量（整形）而不是角速度（浮点型），用角速度 Ki 累计很慢

### S6.2 当前还没真正理解

**ROS2 / Nav2 侧**

* ros2_control 内部如何在 `Update()` 里调度 `read()` / `write()`？controller_manager 的 `update_rate` 与 hardware 的 read/write 时序是怎么对齐的？
* EKF 的状态向量与观测模型（robot_localization 的实现）：x = [pose, twist, accel] 15 维，process model 是什么？jacobian 怎么计算？
* Nav2 行为树（BT）各节点的回调时机？`IsPathValid` 改造为什么能消除高频重规划？
* RPP 算法核心：lookahead point 的选择规则？regulated 在什么时候起作用？

**STM32 / 算法侧**

* Madgwick 算法的四元数迭代怎么推？β 参数的物理意义？
* 增量式 PI vs 位置式 PI 的本质差别（积分饱和、扰动响应）？
* ZUPT（zero-velocity update）的应用条件与数学？

### S6.3 下一步学习路线

1. 实现 Web 界面：在 Web 界面了解机器人当前状态，可设置多点巡航，并支持动态障碍
2. 了解上位机与下位机的串口通信，自行实现
3. 学习 ros2\_control 的实现，能自行实现/配置如三轮全向、四轮麦克纳姆全向
4. 了解别的 SLAM、重定位方案
5. 了解别的全局规划器、局部规划器方案

---

## 附录：跨文档导航索引

* 想看每个阶段做了什么：→ [TIMELINE.md §T1](TIMELINE.md#t1-五个阶段)
* 想从一个新终端把车开起来：→ [TIMELINE.md §T2](TIMELINE.md#t2-复现速查从零开机到-nav2最短路径)
* 想改 PI / 改协议：→ [STM32\_SUMMARY.md](STM32_SUMMARY.md)
* 想调 Nav2：→ [docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)
* 想重装系统：→ [docs/上位机从零配置/香橙派5Pro 从零配置.md](docs/上位机从零配置/香橙派5Pro 从零配置.md)
* 想查命令：→ 各包的 docs/\*\_cheatsheet.md
