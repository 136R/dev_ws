# 移动垃圾桶机器人 — Claude Code 项目上下文

> 本文件是 Claude Code 的核心上下文。每次新对话开始时必须先阅读本文件。 不要假设任何未在此说明的内容，有疑问时优先查阅本文件。

---

## 项目一句话描述

用 ROS2 Humble + Nav2 实现一个移动垃圾桶机器人的​**自主导航**​： 用户在 Web 地图界面点击目标位置 → 机器人自动导航到达。

---

## ⚠️ 工作原则（每次任务前必读）

### 原则 1：分步验证，禁止一次性写完所有代码

每个功能必须拆分成最小可验证单元，逐步实现并通过后再继续：

```
正确做法：步骤1 → 验证通过 → 步骤2 → 验证通过 → 步骤3
错误做法：一次性写完所有代码 → 出错不知道问题在哪
```

**Week 1 强制分步示例：**

1. 创建包骨架 → 验证 `colcon build` 通过
2. 写空的 SystemInterface 类 → 验证插件注册成功
3. 实现 `on_init()` 读 URDF 参数 → 验证参数正确打印
4. 实现 CRC8 函数 → 单独单元测试验证
5. 实现帧打包函数 → 打印 hex 验证帧格式
6. 实现 UART 发送 → 验证 STM32 收到并响应
7. 实现里程计解析 → 验证 `/odom` 数据变化

### 原则 2：以参考代码为基准，不要自己发明

**核心参考资源（按优先级排序）：**

| 优先级 | 资源                                         | 用途                                                   | 链接                                                                                   |
| -------- | ---------------------------------------------- | -------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| ★★★ | `diffdrive_arduino`humble 分支           | ​**my\_bot\_hw 实现模板**​（已验证有完整实现） | https://github.com/joshnewans/diffdrive\_arduino/tree/humble                           |
| ★★★ | Articulated Robotics 真实机器人教程          | 架构理解、调试方法                                     | https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2\_control-real |
| ★★☆ | ros2\_control DiffBot 官方示例（example\_2） | SystemInterface 标准写法                               | https://github.com/ros-controls/ros2\_control\_demos/tree/humble/example\_2            |
| ★★☆ | ros2\_control DiffBot 官方文档               | example\_2 详细说明                                    | https://control.ros.org/humble/doc/ros2\_control\_demos/example\_2/doc/userdoc.html    |
| ★★☆ | ros2\_control 官方文档                       | API 参考                                               | https://control.ros.org/humble/                                                        |

**使用规则：**

* 实现每个新功能前，必须先用 `web_fetch` 读取对应参考代码
* 以参考代码结构为基础修改适配，不要从零设计
* 如果要偏离参考代码，必须说明原因

### 原则 3：遇到配置/插件/传感器问题必须先查文档

调试顺序：`web_fetch 官方文档` → `web_search 社区方案` → 再解释原因

### 原则 4：遇到以下情况必须停下提问，不得自行假设后继续

* ​**涉及未在本文件中说明的硬件参数**​（如新的串口路径、波特率、引脚编号） → 错误示例：自行填写 `/dev/ttyS9` 后继续实现 → 正确做法：列出需要确认的参数，等用户提供后再写代码
* **需要修改 `my_bot` 包中的任何文件** → 无论理由多合理，先停下说明原因，等用户明确授权
* **当前步骤的实现方案与参考代码有重大差异** → 说明差异点和原因，等用户确认方向后再继续
* **不确定某个操作是否会影响已验证通过的步骤** → 宁可多问一次，不要因为一步回退所有已完成进度

---

## 开发环境

| 项目     | 详情                                                                              |
| ---------- | ----------------------------------------------------------------------------------- |
| 宿主机   | Windows 11 + WSL2 Ubuntu 22.04                                                    |
| ROS 版本 | ROS 2 Humble                                                                      |
| 仿真器   | Gazebo Harmonic 8.10.0                                                            |
| GPU      | NVIDIA RTX 4060 Laptop（WSL2 需设置`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`） |
| 工作区   | `~/dev_ws/`                                                                   |
| 仿真包   | `~/dev_ws/src/my_bot`                                                         |
| 硬件包   | `~/dev_ws/src/my_bot_hw`（C++, ament\_cmake）                                 |
| GitHub   | `github.com/136R/my_bot`                                                      |

---

## Claude Code 运行位置与代码同步策略

### Claude Code 在哪里运行

```
仿真开发（已完成）：WSL2 里运行 Claude Code，操作 my_bot 包
硬件开发（当前）：  香橙派上运行 Claude Code，操作 my_bot_hw 包
```

**香橙派上启动 Claude Code 的方式：** 用 VSCode Remote SSH 连接香橙派后，在香橙派终端直接运行 `claude`， 此时 Claude Code 的所有文件操作和命令执行都在香橙派本地完成， 无需通过 SSH 中转，编译、测试全部原地进行。

### 代码同步策略（两包分离，互不干扰）

```
WSL2                          香橙派
my_bot（仿真，禁止修改）        my_bot_hw（硬件，主战场）
     │                              │
     └──────── GitHub ──────────────┘
                  ↑
         只在需要共享配置时使用
         （如 URDF 的 ros2_control.xacro）
```

* `my_bot_hw` 从一开始就在香橙派上创建和开发，**不需要先在 WSL 开发再同步**
* WSL 端无需克隆 `my_bot_hw`，香橙派无需克隆完整 WSL 工作区
* 如果 `my_bot_hw` 需要引用 `my_bot` 里的 URDF 文件，通过 GitHub 拉取即可

---

## 硬件栈（真实机器人）

| 组件       | 规格                                                                            |
| ------------ | --------------------------------------------------------------------------------- |
| 主控       | 香橙派 5 Pro，Ubuntu 22.04 Server，ROS2 Humble（fishros 安装）                  |
| 主控 IP    | `192.168.16.209`（手机热点），WiFi MAC:`54:78:c9:d7:0d:58`              |
| 驱动板     | STM32F303 集成板，UART 通信（**非**micro-ROS）                            |
| 雷达       | 思岚 C1                                                                         |
| 电机       | 25GA-370，减速比 1:20，编码器 11 PPR →​**880 脉冲/圈**​（AB 相四倍频） |
| 车轮直径   | \~65mm（待物理测量确认）                                                        |
| 轮距       | \~171mm（待物理测量确认）                                                       |
| 机体尺寸   | \~30×30×15cm，\~5kg                                                           |
| 驱动方式   | 两轮差速，允许后退                                                              |
| 最大线速度 | 0.4 m/s                                                                         |
| 最大角速度 | 1.2 rad/s                                                                       |
| 工作环境   | 室内，10\~30m 多房间                                                            |

---

## STM32 串口协议

> 厂商固件已实现：电机 PID、差速运动学、编码器读取、IMU 解析。 我们的工作是通过此协议与固件通信，​**不需要重新实现底层控制**​。 厂商代码位于 `dev_ws/docs/vendor/`，修改前必须先阅读。

### 帧格式

```
[0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]
```

* CRC8 多项式：`0x8C`

### 关键指令

| CMD        | 功能                        | 方向          |
| ------------ | ----------------------------- | --------------- |
| `0x01` | 速度指令（线速度 + 角速度） | 主控 → STM32 |
| `0x11` | 请求里程计数据              | 主控 → STM32 |
| `0x12` | 里程计回复                  | STM32 → 主控 |
| `0x13` | 请求原始 IMU 数据           | 主控 → STM32 |
| `0x14` | IMU 数据回复                | STM32 → 主控 |

---

## 软件架构

```
Web 前端 (浏览器地图点击)
    │
    ▼
rosbridge (WebSocket)
    │
    ▼
Nav2 Goal → 路径规划 → MPPI 控制器
    │
    ▼
twist_mux (速度仲裁)
  ├─ 键盘遥控 priority=90
  └─ Nav2 cmd_vel priority=70
    │
    ▼
velocity_smoother → cmd_vel → ros2_control
    │
    ▼
my_bot_hw (C++ 硬件接口插件)
    │  UART
    ▼
STM32F303（电机 PID + 里程计）
```

### 包结构

| 包名            | 用途                                       | 修改权限           |
| ----------------- | -------------------------------------------- | -------------------- |
| `my_bot`    | 仿真专用（URDF、Gazebo、Nav2 仿真配置）    | **禁止修改** |
| `my_bot_hw` | 真实机器人硬件接口（ros2\_control plugin） | 主要工作包         |

---

## my\_bot\_hw 实现规范

### 强制：实现前必须读取参考代码

```
# 每次开始 my_bot_hw 相关任务，按需参考（制定计划时根据任务决定是否读取）：

# 主参考：官方 ros2_control_demos example_2 DiffBot 硬件接口实现（Humble 分支，已验证可访问）
web_fetch https://github.com/ros-controls/ros2_control_demos/tree/humble/example_2/hardware/diffbot_system.cpp

# 补充参考：官方 example_2 文档（包含完整验证步骤和接口说明）
web_fetch https://control.ros.org/humble/doc/ros2_control_demos/example_2/doc/userdoc.html
```

> ⚠️ 说明：diffdrive\_arduino humble 分支的 raw 直链因文件结构与 main 不同而 404， 上方 ros2\_control\_demos 官方源码是等效替代，内容覆盖： on\_init / on\_configure / on\_activate / read / write 完整实现模式。

### SystemInterface 生命周期（以 diffdrive\_arduino 为模板）

```cpp
on_init()       // 从 URDF <param> 读取配置（wheel名称、串口、波特率、CPR等）
on_configure()  // 打开串口连接，初始化通信对象
on_activate()   // 开始收发数据
on_deactivate() // 停止通信但不断开
on_cleanup()    // 关闭串口
on_shutdown()   // 资源释放

read()   // 发送 CMD 0x11 → 解析 CMD 0x12 → 更新 state_interfaces（位置、速度）
write()  // 读 command_interfaces 目标速度 → 打包 CMD 0x01 → UART 发送
```

### URDF ros2\_control 参数块

```xml
<ros2_control name="MyBotHardware" type="system">
  <hardware>
    <plugin>my_bot_hw/MyBotHardware</plugin>
    <param name="left_wheel_name">left_wheel_joint</param>
    <param name="right_wheel_name">right_wheel_joint</param>
    <param name="serial_device">/dev/ttyS9</param>  <!-- ⚠️ 待确认：香橙派实际串口路径，用 ls /dev/ttyS* 或 ls /dev/ttyUSB* 查看 -->
    <param name="baud_rate">115200</param>
    <param name="timeout_ms">1000</param>
    <param name="enc_counts_per_rev">880</param>
  </hardware>
  <!-- joints 同仿真版本 -->
</ros2_control>
```

### 文件结构（参考 diffdrive\_arduino 组织方式）

```
my_bot_hw/
├── CMakeLists.txt
├── package.xml
├── my_bot_hw.xml              # 插件描述文件
├── include/my_bot_hw/
│   ├── my_bot_hardware.hpp    # SystemInterface 头文件
│   └── stm32_comm.hpp         # 串口通信封装
└── src/
    ├── my_bot_hardware.cpp    # SystemInterface 实现
    └── stm32_comm.cpp         # 帧打包、CRC8、UART 读写
```

---

## Week 1 分步实现计划

> 每步完成后，必须执行验证指令确认通过，才能进入下一步。

### 步骤 1：包骨架

```bash
# 验证
cd ~/dev_ws && colcon build --packages-select my_bot_hw
# 期望：build 成功，无错误
```

### 步骤 2：空 SystemInterface 插件注册

```bash
# 验证方式 A：用 pluginlib 工具检查插件是否可被发现（不需要运行 controller_manager）
ros2 run pluginlib_tutorials pluginlib_test  # 如无此工具用方式 B

# 验证方式 B（推荐）：写一个最小 URDF + 最小 launch，启动 ros2_control_node
# 在终端观察输出，期望看到：
# [INFO] [ros2_control_node]: Loading hardware interface plugin 'my_bot_hw/MyBotHardware'
# 不应出现：[ERROR] ... Could not load class 'my_bot_hw/MyBotHardware'

# 插件加载成功后，再运行以下命令确认：
ros2 control list_hardware_interfaces
# 期望：左右轮的 velocity command interface 和 position/velocity state interface 出现在列表里
```

### 步骤 3：on\_init() 读取参数

```bash
# 验证（在 on_init 里 RCLCPP_INFO 打印所有参数）
# 期望：终端看到 serial_device、baud_rate、enc_counts_per_rev=880 等正确值
```

### 步骤 4：CRC8 函数

```bash
# 验证：写独立 test_crc 程序，对照厂商代码已知帧验证 CRC 值
# 不要跳过——CRC 错误会导致通信静默失败
```

### 步骤 5：帧打包函数

```bash
# 验证：打印帧的 hex，与厂商代码注释中示例帧对照
```

### 步骤 6：UART 发送速度指令

```bash
# 验证
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
# 期望：电机实际转动
```

### 步骤 7：里程计读取

```bash
# 验证
ros2 topic echo /odom --once
# 期望：手推车轮时 position 数值变化
```

---

## 当前进度

* [x] URDF 设计
* [x] Gazebo DiffDrive 仿真
* [x] ros2\_control 集成（gz\_ros2\_control 源码编译）
* [x] GPU 雷达仿真（Harmonic gpu\_lidar）
* [x] slam\_toolbox 建图（online async）
* [x] Nav2 仿真跑通（SmacPlanner2D + MPPI + twist\_mux）
* [x] 已保存地图：`~/dev_ws/src/my_bot/config/slam/my_map_serial`
* [x] 香橙派 5 Pro 系统配置（WiFi、SSH、ROS2）
* [ ] **Week 1 步骤 1：包骨架** ← 从这里开始
* [ ] Week 1 步骤 2-7：串口通信
* [ ] Week 2：真机 Nav2
* [ ] Week 3：rosbridge Web 界面
* [ ] Week 4：全系统集成

---

## 已知坑与关键经验

1. ​**slam\_toolbox Humble 参数名**​：`slam_params_file`，不是 `params_file`
2. ​**仿真 ≠ 真机**​：Nav2 仿真参数不能直接用于真机
3. ​**定位精度上限**​：`resolution: 0.05` → \~2-3cm；更高精度用 `resolution: 0.02`
4. ​**WSL2 GPU 渲染**​：每次 `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`
5. ​**diffdrive\_arduino 编译依赖**​：humble 分支除了 `sudo apt install libserial-dev`，CMakeLists 里还 `find_package(serial)`，如果报找不到 serial 包，需要同时克隆 Josh fork 的 serial 库到工作区：`git clone https://github.com/joshnewans/serial`
6. ​**插件注册**​：`my_bot_hw.xml` 的 class name 必须与 `PLUGINLIB_EXPORT_CLASS` 宏完全一致

---

## 禁止事项

* ❌ 不要修改 `my_bot` 包（仿真包，保持稳定）
* ❌ 不要使用 micro-ROS（已确认用自定义 UART 协议）
* ❌ 不要假设编码器 PPR——已确认是 **880 脉冲/圈**
* ❌ 不要一次性生成整个模块的完整代码，必须分步实现
* ❌ 不要在未读参考代码的情况下设计接口

