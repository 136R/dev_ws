# STM32 下位机专题

> 定位：自制差速底盘的 STM32 固件总览。[TIMELINE.md](TIMELINE.md) 与 [SUMMARY.md](SUMMARY.md) 中涉及下位机的地方都链向这里。
> 范围：固件结构 / 串口协议 / 烧录调试 / PI 调试。
> 不在范围：硬件电路设计、PCB 焊接。

---

## M0. 引子

* 硬件：STM32F303RBT6 + ICM-42688-P 6 轴 IMU + 自制差速底盘
* 工程：STM32CubeIDE，位于 [firmware/my\_robot\_stm32/](firmware/my_robot_stm32/)
* 构建：不参与 colcon build；通过 CubeIDE 编译烧录
* 运行频率：主控制环 100 Hz（TIM6 ISR，CONTROL\_DT = 0.01s）
* 与上位机连接：UART1（115200 baud） ↔ 上位机 /dev/ttyS7

> **阅读说明**：本文档中固件实现细节（串口协议解码、PI 调试方法、中断时序、烧录流程等）由 AI 根据源码生成，未经本人深入验证。如有疑问请以源码为准。

---

## M1. 固件结构（模块清单）

源码位于 [firmware/my\_robot\_stm32/Core/{Inc,Src}/app/](firmware/my_robot_stm32/Core/)。


| 模块              | 文件                                                                         | 职责                                  | 关键参数所在                          |
| ----------------- | ---------------------------------------------------------------------------- | ------------------------------------- | ------------------------------------- |
| comm\_protocol    | [comm\_protocol.{c,h}](firmware/my_robot_stm32/Core/Inc/app/comm_protocol.h) | 与 ROS2 的串口协议（见 §M2）         | COMM\_HEADER\_\*,COMM\_TYPE\_\*       |
| encoder           | encoder.{c,h}                                                                | 编码器读取（增量 → counts）          | ENCODER\_PPR / CPR / GEAR\_RATIO      |
| pid               | pid.{c,h}                                                                    | 增量式 PI 控制器                      | PI\_KP/KI\_LEFT/RIGHT                 |
| motor\_controller | motor\_controller.{c,h}                                                      | 速度环主循环（编码器→PI→PWM）       | CONTROL\_PERIOD\_MS                   |
| motor\_driver     | motor\_driver.{c,h}                                                          | 底层 H 桥 / PWM 输出                  | PWM\_PERIOD、方向反转位               |
| imu               | imu.{c,h}                                                                    | ICM-42688-P 原始数据采集 + 校准       | —                                    |
| fusion (Madgwick) | fusion/、FusionAhrs.c                                                        | Fusion 库的 Madgwick AHRS 解算 → yaw | （Fusion β 等内部参数）              |
| calibration       | calibration.{c,h}                                                            | 前馈自动标定                          | —                                    |
| zupt              | zupt.{c,h}                                                                   | 零速更新（ZUPT），静止时锁陀螺        | —                                    |
| robot\_config     | [robot\_config.h](firmware/my_robot_stm32/Core/Inc/app/robot_config.h)       | 集中常量：PI、CPR、轮径、PWM          | 整文件就是参数                        |
| debug\_uart       | debug\_uart.{c,h}                                                            | 调试串口（走 UART4，与 comm 分离）    | —                                    |
| ws2812            | ws2812.{c,h}                                                                 | LED 状态指示（开机关闭 LED）          | —                                    |

### 现行参数（节选自 [robot\_config.h](firmware/my_robot_stm32/Core/Inc/app/robot_config.h)）

```c
ENCODER_PPR      = 500            ENCODER_CPR (×4) = 2000
GEAR_RATIO       = 34.0           COUNTS_PER_WHEEL_REV = 68000
WHEEL_DIAMETER   = 0.065 m        WHEEL_RADIUS = 0.0325 m
WHEEL_TREAD      = 0.171 m        (车体几何用，差异见下)

CONTROL_FREQ_HZ  = 100            CONTROL_DT = 0.01 s
WHEEL_NO_LOAD    ≈ 31.4 rad/s     WHEEL_MAX ≈ 27.2 rad/s
PWM_PERIOD       = 3600           (TIM2 ARR + 1)

PI_KP_LEFT = 0.54   PI_KI_LEFT = 0.44
PI_KP_RIGHT = 0.56  PI_KI_RIGHT = 0.44
PI_OUTPUT_LIMIT = 3600  (即 100% PWM)

ODOM_LPF_ALPHA   = 0.0  (当前未启用低通)
FF_KV (前馈)     = 0.0  (当前未启用前馈)
方向反转：MOTOR_LEFT_DIR_INVERT=0  MOTOR_RIGHT_DIR_INVERT=1
           ENC_LEFT_DIR_INVERT=1   ENC_RIGHT_DIR_INVERT=1
```

> 注意：WHEEL\_TREAD = 0.171 m（STM32 端）与上位机 wheel\_separation = 0.175 m × multiplier 0.98012 ≈ 0.1715 m（[hw\_controllers.yaml](src/my_bot_hw/config/hw_controllers.yaml#L25-L29)）几乎吻合 —— 是两边各自标定后的一致结果，不是一处改了另一处忘改。

### 调用时序

* main() 启动顺序：HAL_Init → ClockConfig → MX_*_Init → comm_protocol_init → motor_controller_init → imu_init → 主循环（中断驱动）
* 周期触发：TIM6 ISR (100 Hz) → motor_controller_update（内部依次：读编码器 delta → 跑 PI → 输出 PWM → 调 comm_protocol_tick）
* 串口收：DMA + 空闲中断 → comm_protocol_uart_rx_event_handler

---

## M2. 串口通信协议（与 ROS2 的数据格式）

权威源：[comm\_protocol.h](firmware/my_robot_stm32/Core/Inc/app/comm_protocol.h)。本节是结构化解读。

### 帧格式

```
[0xAA] [0x55] [TYPE] [LEN] [DATA × LEN] [XOR]
XOR = TYPE ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]
Byte order: little-endian
```

### 类型与方向


| TYPE          | 方向          | 含义                     | LEN     | 总帧长  |
| ------------- | ------------- | ------------------------ | ------- | ------- |
| 0x01 VEL\_CMD | ROS2 → STM32 | 左右轮目标角速度         | 4 字节  | 9 字节  |
| 0x02 FEEDBACK | STM32 → ROS2 | 编码器 delta + IMU + yaw | 36 字节 | 37 字节 |

### 0x01 VEL\_CMD 载荷（4 字节）


| 偏移 | 字段  | 类型  | 单位                           |
| ---- | ----- | ----- | ------------------------------ |
| 0    | left  | int16 | mrad/s（× 1000 还原为 rad/s） |
| 2    | right | int16 | mrad/s                         |

### 0x02 FEEDBACK 载荷（36 字节）


| 偏移 | 字段                        | 类型       | 单位                                 |
| ---- | --------------------------- | ---------- | ------------------------------------ |
| 0    | left\_delta                 | int32      | counts（自上一帧累计）               |
| 4    | right\_delta                | int32      | counts                               |
| 8    | acc\_x / acc\_y / acc\_z    | int32 × 3 | —                                    |
| 20   | gyro\_x / gyro\_y / gyro\_z | int32 × 3 | —                                    |
| 32   | yaw\_mdeg                   | int32      | millidegree（÷ 1000 还原为 deg）    |

> 磁力计已去除：注释明确 ICM-42688-P 是 6 轴，无磁力计字段。

### 频率与超时

* 上行 (0x02)：50 Hz（在 100 Hz 的 motor\_controller 里每隔一拍发一次）
* 下行 (0x01)：上位机 Stm32SerialHardware 周期发送（频率取决于 ros2\_control update\_rate = 100 Hz）
* 看门狗：500 ms 内未收到 VEL\_CMD 即停车（COMM\_WATCHDOG\_TICKS = 50 × 10 ms）
* RX 缓冲：32 字节 DMA buffer（足够容纳多帧）

---

## M3. 烧录 / 调试流程

详见 STM32CubeIDE 工程（[firmware/my_robot_stm32/](firmware/my_robot_stm32/)）。

---

## M4. PI 调试流程

现行值见 §M1。

### 现行配置的判断

* 增量式 PI：Δu = kp*(e[k]-e[k-1]) + ki*e[k]
* robot_config.h 里有两条注释暴露过去的尝试：
  * 纯增量左：p:0.7, i:0.4，缺点为0可能会超调 → 当前下调到 0.54
  * 加了前馈的纯增量问题：从 0 到 12.3 稳定，但是从 12.3 到 6.15 不稳定 → 当前 FF_KV = 0（前馈未启用）

### 输出限幅与未启用项

* PI_OUTPUT_LIMIT = 3600 即 100% PWM 占空比
* ODOM_LPF_ALPHA = 0 当前不滤波，注释说 α=0.2 → τ≈2.5 ms

---

## M5. 下一步学习

学习路线统一见 [SUMMARY.md §S6.3](SUMMARY.md#s63-下一步学习路线)。

---

## 跨文档链接

* 时间线阶段 ④ 入口：[TIMELINE.md §④](TIMELINE.md#阶段--stm32-下位机)
* ROS2 侧的串口接收（my\_bot\_hw 的 Stm32SerialHardware）：[SUMMARY.md §S3.2](SUMMARY.md#s32-my_bot_hw--ros2_control-硬件接口--串口--ekf)
* IMU 处理位置迁移的决策：[SUMMARY.md §S4.4](SUMMARY.md#s44-imu-处理位置迁移ros2--stm32)
