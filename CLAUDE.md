# 移动垃圾桶机器人

## 项目目标

通过Web界面在SLAM地图上点击位置，让机器人自主导航过来收垃圾。

## 使用场景

- 室内，10-30m多房间环境
- 有人员走动（动态障碍）
- Web界面：看地图 + 点击发送目标点

---

## 硬件配置

### 主控（香橙派5Pro）

- OS：Ubuntu 22.04
- ROS2 Humble
- 串口：/dev/ttyS9（连接STM32，115200波特率）

### 驱动板（STM32F303）

- 通信方式：UART，自定义串口协议
- 电机：25GA-370，减速比1:20，11PPR
- 编码器：AB双相，四倍频后880脉冲/圈
- 参考底盘参数（待实测确认）：
  
  - 轮径：0.065m
  - 轴距：0.171m

### 传感器

- 激光雷达：思岚C1

---

## 软件架构

```
Web前端 → rosbridge(WebSocket) → ROS2
                                  ├── Nav2
                                  ├── slam_toolbox
                                  └── ros2_control
                                        ├── diff_drive_controller
                                        └── my_bot_hw（硬件接口插件）
                                              ↓ UART /dev/ttyS9
                                            STM32（商家固件）
```

### 各层职责

- **STM32**：电机PID、编码器读取、运动学计算（商家固件，几乎不改）
- **my_bot_hw**：ros2_control硬件接口插件，实现串口协议对接
- **diff_drive_controller**：差速运动学，发布/odom
- **Nav2 + slam_toolbox**：建图与自主导航
- **rosbridge**：Web界面与ROS2通信桥梁

---

## STM32串口协议

### 帧格式

```
[0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]
 帧头   长度  设备ID  命令码   数据   保留   校验
```

### CRC8算法

多项式0x8C，参考：docs/vendor/Src/APP/Communication.c，crc8_chk_value函数

### 关键命令

| 方向          | CMD  | 功能       | 帧长 | 数据说明                              |
| ------------- | ---- | ---------- | ---- | ------------------------------------- |
| 香橙派→STM32 | 0x01 | 发送速度   | 12   | LineX + LineY + AngleZ（int16×1000） |
| 香橙派→STM32 | 0x11 | 请求里程计 | 8    | 无数据                                |
| STM32→香橙派 | 0x12 | 回复里程计 | 14   | LineX+LineY+Yaw+AngleZ                |
| 香橙派→STM32 | 0x13 | 请求IMU    | 8    | 无数据                                |
| STM32→香橙派 | 0x14 | 回复IMU    | 38   | Gyro+Accel+Quat                       |

---

## 功能包结构

```
dev_ws/
├── CLAUDE.md
├── docs/
│   └── vendor/          # 商家STM32参考代码（不推送GitHub）
└── src/
    ├── my_bot/          # 仿真包（URDF、Nav2配置、仿真launch）
    ├── my_bot_hw/       # 真实硬件包（待创建）
    │   ├── src/my_bot_hw_interface.cpp
    │   ├── config/controllers.yaml
    │   ├── config/my_bot_hw.ros2_control.xacro
    │   └── launch/hardware.launch.py
    └── gz_ros2_control/ # 仿真用（源码编译）
```

---

## 关键Topic

| Topic      | 说明                                |
| ---------- | ----------------------------------- |
| /scan      | 思岚C1激光数据                      |
| /map       | SLAM地图                            |
| /odom      | 里程计（diff_drive_controller发布） |
| /cmd_vel   | Nav2输出速度指令                    |
| /goal_pose | Web界面发送的导航目标               |

---

## 参考项目

- **diffdrive_arduino**：https://github.com/joshnewans/diffdrive_arduino
  my_bot_hw的实现模板，替换串口协议即可
- **Articulated Robotics真实机器人教程**：
  https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real
- **ros2_control官方DiffBot示例**：
  https://github.com/ros-controls/ros2_control_demos（example_2）

---

## 开发环境

### WSL开发机（Windows 11 + WSL2 Ubuntu 22.04）

- 用途：仿真验证、RViz2可视化、代码开发
- GPU：NVIDIA RTX 4060，需设置：
  `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`
- 工作空间：~/dev_ws/src/my_bot

### 香橙派5Pro（机器人本体）

- 用途：运行Nav2、slam_toolbox、ros2_control、rosbridge
- 工作空间：~/dev_ws/src/

---

## 注意事项

- slam_toolbox launch参数名：`slam_params_file`（非`params_file`）
- ros2_control的robot_base_frame：`base_footprint`
- 里程计精度关键：轮径轴距必须实测，不能估算
- 商家固件USART1（PC4/PC5）对应香橙派/dev/ttyS9

---

## 当前进度

- [x] URDF设计
- [x] Gazebo仿真（gz_ros2_control源码编译）
- [x] 激光雷达（gpu_lidar）
- [x] SLAM（slam_toolbox在线异步建图）
- [x] Nav2仿真验证
- [x] 商家STM32代码分析
- [x] 串口协议确认
- [ ] **当前：创建my_bot_hw硬件接口包**
- [ ] 真实机Nav2迁移
- [ ] Web界面（rosbridge）
- [ ] 全链路联调

