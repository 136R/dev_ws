# my_bot_hw 项目记忆

## 项目概述
- **目标**：用 ROS2 Humble + Nav2 实现移动垃圾桶机器人自主导航
- **主控**：香橙派 5 Pro，Ubuntu 22.04 Server，ROS2 Humble
- **通信**：STM32F303 UART（9600/115200 波特率），自定义协议（非 micro-ROS）
- **编码器**：880 脉冲/圈（AB 相四倍频）
- **参考代码**：[diffdrive_arduino humble 分支](https://github.com/joshnewans/diffdrive_arduino/tree/humble)

## Week 1 进度

### ✅ 步骤 1：包骨架
- 创建 my_bot_hw 包，编译成功
- SystemInterface 空实现（所有方法返回 SUCCESS/OK）

### ✅ 步骤 2：插件注册验证
- 创建最小 URDF（robot_hw.urdf.xacro）、launch、controller 配置
- 验证插件加载成功：`[resource_manager]: Loading hardware 'MyBotHardware'`

### ✅ 步骤 3：参数读取 + 接口 export
- on_init() 读取 URDF 参数：left_wheel_name、right_wheel_name、serial_device、baud_rate、timeout_ms、enc_counts_per_rev
- export_state_interfaces()：绑定 position/velocity state
- export_command_interfaces()：绑定 velocity command
- controller_manager 接口验证通过（无错误）

### ✅ 步骤 4：CRC8 函数单独验证
- 实现：include/my_bot_hw/stm32_comm.hpp + src/stm32_comm.cpp
- 算法：LSB-first，Polynomial 0x8C，初值 0，无最终异或
- 测试：test/test_crc8.cpp，全 10 个测试向量 PASS
- 验证：实现与厂商代码完全一致

### ✅ 步骤 5：帧打包函数
- 核心函数 `pack_frame(cmd, data)`：按协议格式 [0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8] 构建帧
- 命令构建器：
  - `build_cmd_velocity(vx, vy, wz)`：CMD 0x01，线速度/角速度编码为 int16_t × 1000，Big-Endian
  - `build_cmd_request_odom()`：CMD 0x11，7 字节帧，查询里程计数据
  - `build_cmd_request_imu()`：CMD 0x13，7 字节帧，查询原始 IMU 数据
- 测试：4 个新帧打包用例全部 PASS（验证帧格式和 CRC 计算）

## 关键代码路径

### STM32 通信协议
```
帧格式：[0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]
- 0x5A：帧头常数
- LEN：总长度（包含所有字节）
- 0x01：设备 ID
- CMD：命令码
- DATA：可变长载荷
- 0x00：预留字节
- CRC8：校验码（计算范围：[0] 到 [len-2]）
```

### 关键命令
| CMD   | 方向 | 用途 | 数据大小 |
|-------|------|------|---------|
| 0x01  | →    | 速度指令 | 6 B |
| 0x11  | →    | 请求里程 | 0 B |
| 0x12  | ←    | 里程响应 | 8 B |
| 0x13  | →    | 请求原始 IMU | 0 B |
| 0x14  | ←    | IMU 响应 | 34 B |

### my_bot_hw 包结构
```
src/my_bot_hw/
├── include/my_bot_hw/
│   ├── my_bot_hardware.hpp         # SystemInterface 头（Step 3）
│   └── stm32_comm.hpp              # 通信函数声明（Step 4）
├── src/
│   ├── my_bot_hardware.cpp         # 硬件接口实现（Step 3）
│   └── stm32_comm.cpp              # CRC8 + 帧处理（Step 4）
├── test/
│   └── test_crc8.cpp               # CRC8 单元测试（Step 4）
├── config/
│   └── hw_controllers.yaml         # controller_manager 配置
├── urdf/
│   └── robot_hw.urdf.xacro         # 真机 URDF（Step 2）
├── launch/
│   └── hardware.launch.py          # 真机启动脚本（Step 2）
└── CMakeLists.txt / package.xml    # 编译配置
```

## 已验证的配置

### URDF 参数块（robot_hw.urdf.xacro）
```xml
<param name="left_wheel_name">left_wheel_joint</param>
<param name="right_wheel_name">right_wheel_joint</param>
<param name="serial_device">/dev/ttyS9</param>
<param name="baud_rate">115200</param>
<param name="timeout_ms">1000</param>
<param name="enc_counts_per_rev">880</param>
```

### 关节接口（对应 my_bot/description/ros2_control.xacro）
- left_wheel_joint / right_wheel_joint
- command interface：velocity (min:-10, max:10)
- state interface：position, velocity

## 下一步（Step 5+）

- **Step 5**：帧打包函数（pack_frame）
- **Step 6**：UART 发送速度指令，测试 `ros2 topic pub /cmd_vel`
- **Step 7**：里程计读取（read()），验证 `/odom` 数据变化

## 常用命令

```bash
# 编译
cd ~/dev_ws && colcon build --packages-select my_bot_hw

# 启动硬件
ros2 launch my_bot_hw hardware.launch.py

# 测试 CRC8
./install/my_bot_hw/bin/test_crc8

# 查看接口
ros2 control list_hardware_interfaces
```

## 关键坑点

1. **CRC8 不匹配**：STM32 厂商固件在行 234 检查 CRC，不匹配会直接拒收帧（`return -1`）
2. **轮参数待测**：轮距 171mm、轮半径 32.5mm 是估值，需物理测量确认
3. **串口路径**：/dev/ttyS9 需根据香橙派实际硬件验证（`ls /dev/ttyS*`）
4. **界面名称必须一致**：关节名 left_wheel_joint/right_wheel_joint 在 URDF、controller 中必须完全一致
