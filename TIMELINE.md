# TIMELINE：从零到 Nav2 实机跑通

> 定位：从零开始到实机 Nav2 跑通的旅程笔记，按阶段写"做了什么 / 关键决策 / 主要产物"。
> 目的是未来的自己按图索骥能复现。原理细节不展开，深入解析见 [SUMMARY.md](SUMMARY.md)，下位机相关见 [STM32\_SUMMARY.md](STM32_SUMMARY.md)。

---

## 三档文档分工


| 文档                                                                                     | 定位            | 何时翻                                   |
| ---------------------------------------------------------------------------------------- | --------------- | ---------------------------------------- |
| TIMELINE.md（本篇）                                                                      | 旅程 + 复现速查 | 想"重做一遍"或回忆"我那时是怎么走过来的" |
| [SUMMARY.md](SUMMARY.md)                                                                 | 按子系统深入    | 想理解"为什么这么做 / 原理是什么"        |
| [STM32\_SUMMARY.md](STM32_SUMMARY.md)                                                    | 下位机固件专题  | 改 PI / 改协议 / 排查下位机问题          |
| [docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)                                   | Nav2 调参权威源 | 调 costmap / planner / controller        |
| [docs/上位机从零配置/香橙派5Pro 从零配置.md](docs/上位机从零配置/香橙派5Pro 从零配置.md) | 香橙派环境配置  | 重装系统、初次烧录、串口/网络权限        |
| src/<pkg>/docs/\*\_cheatsheet.md                                                         | 各包命令速查    | 调试命令、参数表、TF 链排查              |

---

## T1. 五个阶段

每阶段固定四要点：做了什么 / 关键决策 / 主要产物 / 链向。

### 阶段 ① 工作空间 + URDF 搭建

做了什么

* 建 \~/dev\_ws colcon 工作空间
* 创建 my\_bot 包，写 description/robot.urdf.xacro 与 mesh
* Gazebo 里调轮子摩擦系数、连杆质量、惯量
* 验证：模型不塌陷、能前进、能原地旋转，TF 链完整（base\_footprint → base\_link → wheel\_\*）

关键决策

* 用 xacro 而非 raw URDF：参数化轮距、轮径，留出 `sim_mode` 切换钩子
* **轮系尺寸来源**：仿真的轮子直径与轮距跟着 articulatedrobotics 教程配，仿真时怎么给，最后再根据实车修改尺寸
* **摩擦系数与质量**：一般调摩擦系数 + 接触刚度即可。判定无误的标准：机器人前进 yaw 不飘、旋转轮子不会死抓地

  ```xml
  <gazebo reference="right_wheel">
      <!-- 摩擦系数 越大摩擦力越高，一般情况下 mu1 = mu2 -->
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>

      <!-- 接触刚度 (Kp, Kd)：一般给当前值即可 -->
      <kp>1000000.0</kp>
      <kd>100.0</kd>

      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <fdir1>1 0 0</fdir1>

      <!-- 可视化材质：在 Gazebo 渲染中显示 -->
      <material>Gazebo/Blue</material>
  </gazebo>
  ```


主要产物

* [src/my\_bot/description/](src/my_bot/description/) —— URDF/xacro + mesh
* [src/my\_bot/launch/rsp.launch.py](src/my_bot/launch/rsp.launch.py) —— robot\_state\_publisher

链向

* 仿真启动细节：[src/my\_bot/docs/sim\_cheatsheet.md](src/my_bot/docs/速查.md)
* 包职责定义：[docs/architecture/package\_layout.md](docs/architecture/package_layout.md)

---

### 阶段 ② 仿真完整闭环

做了什么

* 在 my\_bot 里接入 ros2\_control 的 Gazebo 仿真插件（gaz\_ros2\_ctl\_use\_sim.yaml、my\_controllers.yaml）
* 用 [src/my\_bot/launch/launch\_sim.launch.py](src/my_bot/launch/launch_sim.launch.py) 一键拉起：Gazebo + spawn + diff\_drive + IMU + laser
* 加 EKF（robot\_localization），融合 /odom 与仿真 IMU
* SLAM Toolbox 建图 → 保存地图 → 切定位模式
* Nav2 仿真跑通：teleop 给目标点能到点
* 完成 SLAM/Nav 包拆分：把 SLAM/Nav 从 my\_bot 中独立为 my\_bot\_slam、my\_bot\_nav

关键决策

* sim\_mode xacro 参数：开关 Gazebo 差速插件 vs my\_bot\_hw 的 ros2\_control 硬件接口（同一份 URDF 通吃仿真和实机）
* 仿真 IMU 不漂移、不需要校准 —— 与实机的差别会在阶段 ⑤ 反过来咬人
* **仿真期最大的卡壳**（详见 [SUMMARY §S2 仿真期已经踩过的坑](SUMMARY.md#仿真期已经踩过的坑)）：
  * articulatedrobotics 的教程一般到 slam toolbox 前都没大问题；到 slam_toolbox 阶段卡过 `ros2 topic echo /scan` 卡死（WSL 镜像模式 → 换 CycloneDDS）、humble 带参 API 是 `slam_params_file` 不是 `params_file`
  * nav2 默认 DWB 在 < 50 cm 通道不够用，需换 RPP

主要产物

* [src/my\_bot/config/{ekf\_sim,my\_controllers,gz\_bridge,laser\_filters\_sim,twist\_mux}.yaml](src/my_bot/config/)
* [src/my\_bot\_slam/launch/slam.launch.py](src/my_bot_slam/launch/slam.launch.py)、[nav\_slam.launch.py](src/my_bot_slam/launch/nav_slam.launch.py)
* [src/my\_bot\_nav/launch/nav.launch.py](src/my_bot_nav/launch/nav.launch.py)

链向

* 仿真子系统深入：[SUMMARY.md §S2](SUMMARY.md#s2-仿真章节独立一节)
* Nav2 调参开始的地方：[docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)

---

### 阶段 ③ 香橙派环境配置

做了什么

* 烧录系统镜像 → 首次启动 → 联网
* 启用 UART1（设备树 overlay：uart1-m1）
* 鱼香 ROS2 一键安装 Humble
* 装项目依赖（laser\_filters / twist\_mux / robot\_localization / nav2 / slam\_toolbox）
* 配 SSH 与 GitHub，克隆代码到 \~/dev\_ws

关键决策

* 串口分配：STM32 通信走 `/dev/ttyS7`（UART7-M2），LiDAR 走 `/dev/ttyUSB0`（C1，460800 波特率）—— 见 [robot_bringup.launch.py:17-21](src/my_bot_hw/launch/robot_bringup.launch.py#L17-L21)
* **UART 选择**：看驱动板的端口需求接线即可（没纠结哪一组 UART 映射，按板子走线决定）
* **权限**：给过 USB 权限即可（dialout 组 / udev 规则未深入配，热插拔可能会变设备号）

主要产物

* 环境配置全过程：[docs/上位机从零配置/香橙派5Pro 从零配置.md](docs/上位机从零配置/香橙派5Pro 从零配置.md)
* 工作空间同步流程：[docs/workflows/rsync\_workflow.md](docs/workflows/rsync_workflow.md)

链向

* 环境配置权威源：[docs/上位机从零配置/香橙派5Pro 从零配置.md](docs/上位机从零配置/香橙派5Pro 从零配置.md)（缺口清单见文末）
* 烧录系统/串口权限/网络等所有细节，都在那份文档里

---

### 阶段 ④ STM32 下位机

TIMELINE 这里只一段概括，细节全部在[STM32\_SUMMARY.md](STM32_SUMMARY.md)。

做了什么

* 选板：自制电路板 + STM32F303RBT6 + ICM-42688-P IMU
* 实现：编码器读取、PI 速度环、电机驱动、IMU 采集
* IMU：用 Fusion 库的 Madgwick 算法在 STM32 端解算 yaw，避开了在 ROS2 端做 imu\_calib/bias/死区仍漂移的问题（360° 误差 ≈ 0.5°）
* 设计串口协议（详见 [STM32\_SUMMARY.md §M2](STM32_SUMMARY.md#m2-串口通信协议与-ros2-的数据格式)）：
  * 帧格式：[0xAA] [0x55] [TYPE] [LEN] [DATA] [XOR]，小端
  * 上行 STM32→ROS2：左右轮 delta + acc(xyz) + gyro(xyz) + yaw\_mdeg，50 Hz
  * 下行 ROS2→STM32：左右轮目标速度 mrad/s（int16），看门狗 500 ms
* ROS2 侧的串口收发实现在 my\_bot\_hw 的 Stm32SerialHardware（ros2\_control SystemInterface 插件）

关键决策

* IMU 处理位置：先在 ROS2 试过 imu\_calib 六面标定 + bias + 死区，仍动态漂移；移到 STM32 用 Madgwick 之后稳定（详见 [SUMMARY.md §S4.4](SUMMARY.md#s44-imu-处理位置迁移ros2--stm32)）
* 50 Hz 上行频率：与 EKF 100 Hz 融合频率匹配（差一倍是足够的，再快只是浪费带宽）

主要产物

* 固件源码：[firmware/my\_robot\_stm32/Core/](firmware/my_robot_stm32/Core/)
* 协议头：[firmware/my\_robot\_stm32/Core/Inc/app/comm\_protocol.h](firmware/my_robot_stm32/Core/Inc/app/comm_protocol.h)
* 参数集：[firmware/my\_robot\_stm32/Core/Inc/app/robot\_config.h](firmware/my_robot_stm32/Core/Inc/app/robot_config.h)

链向

* [STM32\_SUMMARY.md](STM32_SUMMARY.md)（固件结构 / 协议 / 烧录 / PI 调试 全在那）

---

### 阶段 ⑤ 上位机 ROS2 实机闭环

做了什么

* 创建 my\_bot\_hw 包，写 Stm32SerialHardware（ros2\_control 的 SystemInterface 插件）
* 衔接：xacro sim\_mode:=false → 加载 ros2\_control\_hw.xacro → 串口对接 STM32
* 启用三个控制器：diff\_cont（DiffDriveController）、joint\_broad、imu\_broad
* 接入 EKF：仅用 /odom 的 vx + /imu\_broad/imu 的 yaw（仿真期是融合全部 IMU 字段，实机砍掉了 roll/pitch）
* 标定轮系几何：wheel\_separation\_multiplier=0.98012、right\_wheel\_radius\_multiplier=0.99
* 调 SLAM Toolbox 参数（实机版本与仿真不同），落到 mapper\_params\_hw.yaml
* 调 Nav2：试过 DWB / MPPI 都不理想，最终落在 ThetaStar + RPP，并改 BT 用 IsPathValid 替代周期重规划
* 跑通：robot\_bringup → nav\_slam → 发目标点 → 实机到点

关键决策

* EKF 的 odom0\_config 只勾 vx，imu0\_config 只勾 yaw —— 让"编码器管位移，IMU 管朝向"，避免两边都给 yaw 冲突
* enable\_odom\_tf: false 在 diff\_cont 端，odom→base\_footprint TF 由 EKF 唯一发布（防双源 TF）
* planner 选 ThetaStar 而非 SmacPlanner2D/NavFn：后两者路径"离障碍多远"难调（详见 [调参素材](docs/调参素材/调参素材.md#global_costmap全局规划器)）
* controller 选 RPP：DWB 不跟全局/转弯失败，MPPI 出现"倒车入库""歪着走"等异常（详见 [调参素材](docs/调参素材/调参素材.md#local_costmap局部规划器)）
* **仿真到实机迁移踩的坑**：雷达外参（`lidar_yaw`）没标定，雷达扫到的直线有点歪

主要产物

* [src/my\_bot\_hw/launch/robot\_bringup.launch.py](src/my_bot_hw/launch/robot_bringup.launch.py)
* [src/my\_bot\_hw/config/hw\_controllers.yaml](src/my_bot_hw/config/hw_controllers.yaml)、[ekf\_hw.yaml](src/my_bot_hw/config/ekf_hw.yaml)、[laser\_filters.yaml](src/my_bot_hw/config/laser_filters.yaml)
* [src/my\_bot\_slam/config/mapper\_params\_hw.yaml](src/my_bot_slam/config/mapper_params_hw.yaml)
* `src/my_bot_nav/config/nav2_params_hw.yaml` *(当时的 DWB 参数，现已删除；当前用 `config/hw/nav2_params_hw_{rpp,neupan}.yaml`)*

链向

* 各子系统的"为什么这么做"：[SUMMARY.md §S3](SUMMARY.md#s3-主题深入按子系统实机为主)
* Nav2 调参细节：[docs/调参素材/调参素材.md](docs/调参素材/调参素材.md)

---

## T2. 复现速查：从零开机到 Nav2（最短路径）

> 适用场景：仓库与硬件都齐了，从一个新终端打开，到 Nav2 跑起来。
> 完整命令变体在 [docs/workflows/runbook.md](docs/workflows/runbook.md)。

### 步骤 0：环境准备（每次开新终端）

```bash
cd ~/dev_ws
source install/setup.bash
```

代码更新或首次进入时编译：

```bash
colcon build --packages-select my_bot my_bot_hw my_bot_slam my_bot_nav sllidar_ros2 --symlink-install
source install/setup.bash
```

验证：ros2 pkg list | grep my\_bot 应列出四个包。

### 步骤 1：实机底盘 bringup

```bash
ros2 launch my_bot_hw robot_bringup.launch.py
```

> 这一条 launch 启动了：ros2\_control（含 Stm32SerialHardware 串口连 STM32）、三个 controller spawner、SLAMTEC C1、laser\_filters、twist\_mux、odom\_relay、EKF。

验证：

```bash
ros2 topic echo /odom --once          # 来自 diff_drive_controller，应有 twist.twist.linear.x
ros2 topic echo /imu_broad/imu --once # 来自 imu_broad（实际数据来自 STM32 Madgwick）
ros2 topic echo /scan --once           # 来自 sllidar_node
ros2 topic echo /scan_filtered --once  # 经 laser_filters 后
ros2 run tf2_ros tf2_echo odom base_footprint  # 由 EKF 发布
```

排查清单参见 [src/my\_bot\_hw/docs/robot\_hw\_cheatsheet.md](src/my_bot_hw/docs/速查.md)。

### 步骤 2：建图（首次跑或地图改变后）

```bash
# 终端 2
ros2 launch my_bot_slam slam.launch.py mode:=mapping

# 终端 3：手动遥控建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard
```

验证：RViz 中加载 /map，能看到逐步生长的栅格地图。建图完成后保存：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/dev_ws/src/my_bot_slam/config/maps/my_map
```

> 已建好的地图就跳过本步，直接走步骤 3。

### 步骤 3：定位 + 导航（日常使用）

```bash
# 终端 2：定位 + Nav2 一起
ros2 launch my_bot_slam nav_slam.launch.py slam_mode:=localization
```

验证：

```bash
ros2 topic echo /plan --once   # ThetaStar 输出，应有 poses 列表
```

RViz 加载 /map + /scan\_filtered + /plan + /local\_costmap/costmap + /global\_costmap/costmap，确认机器人位置在地图中正确。

### 步骤 4：发送目标点

* RViz："2D Goal Pose" 工具点击地图
* 命令行：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

期望行为：

1. /plan 出现一条 ThetaStar 规划的折线
2. /cmd\_vel\_nav 开始发布速度命令
3. 经 twist\_mux → /diff\_cont/cmd\_vel\_unstamped → STM32 → 轮子转动
4. 到点后停车

### 速查表：常用调试命令


| 想看什么           | 命令                                            |
| ------------------ | ----------------------------------------------- |
| TF 树              | ros2 run tf2\_tools view\_frames                |
| 节点关系图         | rqt\_graph                                      |
| 控制器状态         | ros2 control list\_controllers                  |
| 当前 cmd\_vel 来源 | ros2 topic echo /diff\_cont/cmd\_vel\_unstamped |
| EKF 输出           | ros2 topic echo /odometry/filtered              |
| 静态参数           | ros2 param dump /<node\_name>                   |

更多细节见各包 cheatsheet：[my\_bot\_hw](src/my_bot_hw/docs/速查.md) / [my\_bot\_slam](src/my_bot_slam/docs/速查.md) / [my\_bot\_nav](src/my_bot_nav/docs/速查.md)。
