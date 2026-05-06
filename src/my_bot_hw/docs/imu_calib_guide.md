# IMU 加速度计标定指南（imu_calib）

> 工具包：[Nathan85001/imu_calib](https://github.com/Nathan85001/imu_calib)（ros2 分支，需从源码编译）
> 标定输出：`config/imu_accel_calib.yaml`
> 参考来源：[imu_calib README](https://github.com/Nathan85001/imu_calib/blob/ros2/README.md)、[dpkoch/imu_calib Issue #28](https://github.com/dpkoch/imu_calib/issues/28)、[ICM-42688-P Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)

---

## 一、标定什么，为什么要标定

### 1.1 IMU 误差的来源

一块 IMU 出厂后，即使在完全静止的状态下，加速度计的读数也不会精确输出 `[0, 0, 9.81]`。原因有三类：

```
真实加速度 a_true
        │
        ▼
 传感器物理测量
        │
        ├─ 零偏（Bias）：静止时读数偏离零  → 如 X 轴读 +0.15 m/s² 而非 0
        ├─ 比例误差（Scale）：灵敏度偏差   → 重力应是 9.81，读出 9.72
        └─ 轴间耦合（Cross-axis）：X 转动影响 Y 读数（通常很小，可忽略）
        │
        ▼
 传感器原始输出 a_raw
```

**imu_calib 能修正的是：加速度计零偏 + 比例误差，并使用 3×3 修正矩阵吸收一部分轴间耦合 / 安装不正交误差。**

### 1.2 未标定时有什么影响


| 误差类型         | 对你的机器人的影响                                                               |
| ---------------- | -------------------------------------------------------------------------------- |
| 加速度计零偏     | EKF 的线速度/位置估计有系统性漂移；互补滤波器 pitch/roll 偏几度                  |
| 加速度计比例误差 | 急加速时 IMU 数据与里程计融合出现比例失调，odom 跳变                             |
| 陀螺仪零偏       | 你当前方案是在 ROS 层做启动时均值标定（`apply_calib_node` 的 `calibrate_gyros`） |

### 1.3 何时不需要做

如果你的机器人满足以下所有条件，**可以跳过加速度计标定**：

- 只用于室内建图/导航，速度 ≤ 0.2 m/s，无剧烈急加速
- EKF 以里程计为主、IMU 为辅（`ekf_hw.yaml` 中 IMU 协方差偏保守）
- `/imu/data` 静止时 `linear_acceleration.z` 在 `9.75~9.87 m/s²` 范围内

**若只是偶尔导航建图，当前代码已够用，标定是锦上添花。**

---

## 二、imu_calib 的工作原理

### 标定数学模型

标定的目标是求出从"原始读数"到"真实值"的线性修正。当前 `ros2` 版本输出的是 **3×3 加速度计修正矩阵 + 3 维 bias**：

```
a_corrected = accel_scale * (a_raw - accel_bias)
```

其中：

- `accel_bias`：3 个轴的零偏
- `accel_scale`：3×3 修正矩阵
  - 对角项主要对应各轴比例误差
  - 非对角项可吸收一部分轴间耦合 / 安装不正交误差

**求解方式**：将传感器静止放置在 6 个不同姿态，每次都有一个轴对齐重力方向（+g 或 -g），另外两轴应读 0。6 个位置 × 多次采样后构成超定方程组，最小二乘求解。

```
位置示例（不要求精确对齐，大约即可）：

  位置 1: Z 轴朝上   → az ≈ +9.81, ax ≈ 0, ay ≈ 0
  位置 2: Z 轴朝下   → az ≈ -9.81, ax ≈ 0, ay ≈ 0
  位置 3: X 轴朝上   → ax ≈ +9.81, ay ≈ 0, az ≈ 0
  位置 4: X 轴朝下   → ax ≈ -9.81, ay ≈ 0, az ≈ 0
  位置 5: Y 轴朝上   → ay ≈ +9.81, ax ≈ 0, az ≈ 0
  位置 6: Y 轴朝下   → ay ≈ -9.81, ax ≈ 0, az ≈ 0
```

> **关键认知**：当前 `ros2` 版本 **不会自动检测姿态方向**，而是按固定顺序提示 `X+ → X- → Y+ → Y- → Z+ → Z-`。你需要明确知道 IMU 坐标系方向，并按提示让对应轴朝上。

---

## 三、安装（从源码编译）

imu_calib 没有 ROS2 Humble 的二进制包，需要自行编译：

```bash
# 1. 进入工作空间 src 目录
cd ~/dev_ws/src

# 2. 克隆 ros2 分支
git clone https://github.com/Nathan85001/imu_calib.git -b ros2

# 3. 安装依赖
cd ~/dev_ws
rosdep install --from-paths src -r -y

# 4. 编译
colcon build --packages-select imu_calib --symlink-install

# 5. 加载环境变量
source install/setup.bash
```

验证安装成功：

```bash
ros2 pkg list | grep imu_calib   # 应输出 imu_calib
ros2 run imu_calib do_calib --help
```

---

## 四、标定前准备

### 4.1 启动机器人（仅需 IMU 数据，不需要导航）

```bash
# 终端 1：启动底层硬件（STM32 通信 + IMU 广播）
ros2 launch my_bot_hw robot_bringup.launch.py

# 终端 2：确认 IMU 数据正常
ros2 topic hz /imu_broad/imu         # 应约 100 Hz
ros2 topic echo /imu_broad/imu --once # 确认有 linear_acceleration 数据
```

正常的静止状态 IMU 输出示例：

```
linear_acceleration:
  x: 0.15    # 小的偏置，正常
  y: -0.08
  z: 9.72    # 应接近 9.81，偏差在 ±0.3 内都正常
```

如果 `linear_acceleration.z` 偏差 > 0.5 m/s²（如读出 9.2），说明比例误差明显，**标定收益较大**。

### 4.2 确定 IMU 坐标系方向

在开始之前，需要知道 ICM-42688-P 在你的机器人上的安装方向，以便判断哪个面朝上对应哪个轴。

```bash
# 将机器人正常放置（轮子朝下），查看哪个轴读重力
ros2 topic echo /imu_broad/imu --field linear_acceleration --once
```

预期：某一个轴会接近 `+9.81` 或 `-9.81 m/s²`，取决于 ICM-42688-P 的安装方向以及你 STM32 中是否做了轴映射。

记录下来：**正常放置时，哪个轴读重力？正号还是负号？** 这一步很重要，因为 `do_calib` 会明确要求你摆出 `X+ / X- / Y+ / Y- / Z+ / Z-` 六个姿态。

---

## 五、标定流程（step-by-step）

### 5.1 启动标定节点

```bash
# 终端 3：启动标定节点，重映射输入话题
ros2 run imu_calib do_calib \
  --ros-args \
  -r /imu:=/imu_broad/imu \
  -p output_file:=$(ros2 pkg prefix my_bot_hw)/share/my_bot_hw/config/imu_accel_calib.yaml
```

> **注意**：`output_file` 路径是标定结果保存的位置。这里先保存到 install 目录，标定完成后再手动复制到源码目录。

更简单的做法——直接指定完整路径：

```bash
ros2 run imu_calib do_calib_node \
  --ros-args \
  -r /imu:=/imu_broad/imu \
  -p output_file:=/home/orangepi/dev_ws/src/my_bot_hw/config/imu_accel_calib.yaml
```

### 5.2 执行 6 个姿态

标定节点启动后会在终端输出类似：

```
[INFO] Waiting for IMU data...
[INFO] Press Enter when you are ready to begin calibration...
```

**按回车后，节点会按固定顺序要求你依次摆放 6 个姿态：`X+ → X- → Y+ → Y- → Z+ → Z-`。每个姿态按一次回车后，程序开始连续采样。**

这里最容易让人困惑的是：`X+ / X- / Y+ / Y-` 到底该怎么摆？

先记住一句话：

> **节点提示 `X+`，意思就是“让 IMU 的 `+X` 方向朝向天花板”。**
>
> 节点提示 `X-`，意思就是“让 IMU 的 `-X` 方向朝向天花板”。
>
> `Y+ / Y- / Z+ / Z-` 同理。

### 先用你的实测结果确定 `Z` 轴

你前面已经测到机器人正常放置时：

```bash
ros2 topic echo /imu_broad/imu --field linear_acceleration --once
x: 0.12
y: -0.225
z: 9.797
```

这说明正常放地上时，`z ≈ +9.8`，也就是：

- `Z+`：机器人正常放置，轮子朝下
- `Z-`：把机器人整个翻过来，轮子朝上

这两个姿态你已经可以直接做。

### `X`、`Y` 轴怎么判断

你还需要再做两次简单测试，确认 `X`、`Y` 分别对应机器人哪一面：

```bash
# 让机器人“前脸朝上”立起来，测一次
ros2 topic echo /imu_broad/imu --field linear_acceleration --once

# 再让机器人“左侧朝上”立起来，测一次
ros2 topic echo /imu_broad/imu --field linear_acceleration --once
```

判断规则非常简单：

- 哪一轴接近 `+9.8`，就说明当前是这个轴的“正方向朝上”
- 哪一轴接近 `-9.8`，就说明当前是这个轴的“负方向朝上”

举例：

- 如果机器人“前脸朝上”时，看到 `x ≈ +9.8`
  - 说明这个姿态就是 `X+`
- 如果机器人“前脸朝下”时，看到 `x ≈ -9.8`
  - 说明这个姿态就是 `X-`
- 如果机器人“左侧朝上”时，看到 `y ≈ +9.8`
  - 说明这个姿态就是 `Y+`
- 如果机器人“右侧朝上”时，看到 `y ≈ -9.8`
  - 说明这个姿态就是 `Y-`

### 小白可直接照着摆的理解方式

假设你测完后确认了：

- 前脸朝上 = `X+`
- 前脸朝下 = `X-`
- 左侧朝上 = `Y+`
- 右侧朝上 = `Y-`
- 正常放置 = `Z+`
- 翻过来放 = `Z-`

那 6 个姿态就可以直接理解成：


| 步骤 | 节点提示 | 你该怎么摆机器人           |
| ---- | -------- | -------------------------- |
| 1    | `X+`     | 让机器人前脸朝上           |
| 2    | `X-`     | 让机器人前脸朝下           |
| 3    | `Y+`     | 让机器人左侧朝上           |
| 4    | `Y-`     | 让机器人右侧朝上           |
| 5    | `Z+`     | 机器人正常放地上，轮子朝下 |
| 6    | `Z-`     | 把机器人翻过来，轮子朝上   |

> **如果你还没测清楚 `X/Y` 分别对应哪一面，不要直接开始正式标定。**
>
> 最好先把“哪个面朝上对应哪个轴”记在纸上，再按提示做 `do_calib_node`。

**每个姿态的操作步骤：**

```
①  摆好姿态
②  等待机器人完全静止（约2秒，手不要碰）
③  按回车
④  等这一姿态的采样结束后，再进行下一个姿态
```

> **重要提示**：
>
> - 采集时机器人必须完全静止，轻微抖动不影响，但不能有人走动振动地面
> - 该版本会在每个姿态下连续采样，默认每个姿态采 `500` 个样本；摆好后再按回车
> - 不需要精确 90°，但要尽量让被提示的那个轴朝上

### 5.3 等待计算完成

6 个姿态都采集完成后，节点自动计算并保存结果：

```
[INFO] All positions collected, computing calibration...
[INFO] Calibration complete.
[INFO] Results saved to /home/orangepi/dev_ws/src/my_bot_hw/config/imu_accel_calib.yaml
```

---

## 六、理解标定结果

生成的 `imu_accel_calib.yaml` 内容示例：

```yaml
accel_scale:
  - 1.0023
  - 0.0008
  - -0.0012
  - 0.0005
  - 0.9987
  - 0.0009
  - -0.0010
  - 0.0006
  - 1.0012
accel_bias:
  - 0.1234
  - -0.0456
  - 0.0789
```

### 如何判断标定质量


| 参数                   | 典型正常范围        | 需要关注         | 说明                                                                |
| ---------------------- | ------------------- | ---------------- | ------------------------------------------------------------------- |
| `accel_bias[0..2]`     | ±0.5 m/s²         | > ±1.0          | ICM-42688-P 出厂零偏规格较小，超出通常说明标定时在抖动              |
| `accel_scale` 对角项   | 0.98 ~ 1.02         | < 0.95 或 > 1.05 | 主要反映各轴比例误差                                                |
| `accel_scale` 非对角项 | 接近 0（如 < 0.03） | 数值明显偏大     | 反映轴间耦合 / 安装不正交；过大通常说明六面姿态不准或采样过程不稳定 |

**如果结果异常（bias 很大、对角项偏离 1 很多、非对角项明显偏大），最常见原因是采集时机器人没有静止，或者六个姿态摆得不够准。**

---

## 七、应用标定（将修正加入数据流）

标定完成后，需要在启动时运行 `apply_calib` 节点，将原始 IMU 数据修正后再送给互补滤波器。

### 7.1 数据流变化

```
标定前：
  STM32 → /imu_broad/imu ──────────────────────────→ imu_complementary_filter → /imu/data → EKF

标定后：
  STM32 → /imu_broad/imu → apply_calib → /imu/imu_corrected → imu_complementary_filter → /imu/data → EKF
```

### 7.2 修改 robot_bringup.launch.py

在 launch 文件中添加 `apply_calib` 节点。以下是需要添加的内容（在互补滤波器节点之前）：

```python
# 在 robot_bringup.launch.py 顶部的 import 区域添加：
from pathlib import Path

# 在 Node 列表中添加（放在 imu_complementary_filter 节点之前）：
Node(
    package='imu_calib',
    executable='apply_calib',
    name='apply_calib',
    remappings=[
        ('/raw',       '/imu_broad/imu'),
        ('/corrected', '/imu/imu_corrected'),
    ],
    parameters=[{
        'calib_file': str(Path(get_package_share_directory('my_bot_hw')) / 'config' / 'imu_accel_calib.yaml'),
        'calibrate_gyros': True,
        'gyro_calib_samples': 200,
    }],
),
```

> **为什么这里要设 `calibrate_gyros: True`？**
>
> 你当前的方案是 **STM32 只发布原始 gyro，ROS 层在 `apply_calib_node` 启动时估计陀螺零偏**。因此 bringup 刚启动后的前几秒，机器人必须完全静止，让它完成 gyro bias 标定。

然后将互补滤波器的输入话题从 `/imu_broad/imu` 改为 `/imu/imu_corrected`：

```python
# 找到 imu_complementary_filter 节点，修改 remappings：
remappings=[
    ('imu/data_raw', '/imu/imu_corrected'),   # 修改前是 /imu_broad/imu
    ('imu/data',     '/imu/data'),
],
```

### 7.3 将标定文件加入 CMakeLists.txt

确保 `imu_accel_calib.yaml` 被安装到 share 目录：

```cmake
# 在 my_bot_hw/CMakeLists.txt 的 install(DIRECTORY ...) 部分，
# config/ 目录已经在安装列表中，无需单独添加。
# 确认有如下内容：
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

### 7.4 重新编译并测试

```bash
# 重新编译（因为 yaml 文件已通过 symlink-install 链接，通常无需重新编译）
colcon build --packages-select my_bot_hw --symlink-install
source install/setup.bash

# 启动并验证
ros2 launch my_bot_hw robot_bringup.launch.py
```

---

## 八、验证标定效果

### 8.1 检查修正节点是否运行

```bash
ros2 node list | grep apply           # 应看到 /apply_calib
ros2 topic hz /imu/imu_corrected      # 应约 100 Hz
```

### 8.2 对比标定前后的静止读数

```bash
# 将机器人水平放置，静止不动
# 标定前（原始）
ros2 topic echo /imu_broad/imu --field linear_acceleration --once

# 标定后（修正）
ros2 topic echo /imu/imu_corrected --field linear_acceleration --once
```

**预期改善：**


| 指标           | 标定前          | 标定后（预期）    |
| -------------- | --------------- | ----------------- |
| 静止时`z` 轴   | 如 9.72 m/s²   | 更接近 9.81 m/s² |
| 静止时`x/y` 轴 | 如 ±0.15 m/s² | 更接近 0          |

### 8.3 检查 EKF 输出是否改善

```bash
# 机器人静止放置 30 秒，观察 EKF 位置是否漂移
ros2 topic echo /odometry/filtered --field pose.pose.position
```

标定前静止时位置可能缓慢漂移（累积误差），标定后漂移应减小。

---

## 九、针对你的机器人：标定收益评估

你的机器人参数：

- ICM-42688-P，±16g 量程（灵敏度 2048 LSB/g）
- 最大速度 0.2 m/s，室内 SLAM/Nav2 用途
- EKF 融合里程计 + IMU，odom 权重较高

### 预期收益


| 用途                     | 加速度计标定收益                         | 优先级 |
| ------------------------ | ---------------------------------------- | ------ |
| 直线行驶里程计精度       | 轻微改善（线加速度估计更准）             | 低     |
| 急转弯时 pitch/roll 估计 | 有改善，bias 补偿后互补滤波更准          | 中     |
| 静止时 EKF 漂移          | 明显改善（bias 归零）                    | 高     |
| 陀螺仪偏航漂移           | **无改善**（6 轴固有限制，与此工具无关） | —     |

### 标定文件路径（本项目约定）

```
src/my_bot_hw/config/imu_accel_calib.yaml
```

首次标定完成后，将该文件提交到 git，后续无需重复标定（除非更换传感器或重新焊接）。

---

## 十、常见问题速查


| 现象                                      | 原因                                                         | 解决方法                                                         |
| ----------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------------------- |
| `do_calib` 节点没有收到 IMU 数据          | 话题名称不对                                                 | 用`ros2 topic list` 确认话题，检查 `-r /imu:=` 重映射是否正确    |
| 采样完成但结果很差                        | 采集时传感器在抖动，或六个姿态没有按`X+/X-/Y+/Y-/Z+/Z-` 摆放 | 等待完全静止再按回车，严格按节点提示摆放                         |
| `accel_scale` 对角项超出 `0.95~1.05` 范围 | 姿态不够接近对齐重力                                         | 重新标定，每个姿态尽量让被提示的那个轴更接近竖直                 |
| 标定后`/imu/imu_corrected` 没有数据       | `apply_calib` 节点未启动                                     | 检查 launch 文件是否添加了该节点，检查 calib_file 路径是否存在   |
| EKF 在`apply_calib` 后输出异常跳变        | 互补滤波器输入话题没有切换到修正后的话题                     | 确认`imu_complementary_filter` 订阅的是 `/imu/imu_corrected`     |
| 刚启动时角速度短时间没有输出或姿态未稳定  | `apply_calib` 正在执行 gyro bias 标定                        | 启动后前几秒不要碰机器人，等待日志提示 gyro calibration complete |
| 标定文件`accel_bias` 值超过 ±1.0 m/s²   | 标定时机器人移动或地面振动                                   | 选择更稳固的地面（如地板），关闭附近振动源，重新标定             |
| `colcon build` 后找不到 `imu_calib` 包    | 源码没有放在`src/` 下或没有 `source install/setup.bash`      | 确认`src/imu_calib/` 目录存在，重新 build 后 source              |

---

## 十一、调试命令速查

```bash
# 确认 IMU 原始数据正常
ros2 topic echo /imu_broad/imu --field linear_acceleration --once

# 确认标定修正节点在运行
ros2 node info /apply_calib

# 实时对比修正前后
ros2 topic echo /imu_broad/imu     --field linear_acceleration &
ros2 topic echo /imu/imu_corrected --field linear_acceleration

# 查看当前标定文件内容
cat ~/dev_ws/src/my_bot_hw/config/imu_accel_calib.yaml

# 查看 IMU 互补滤波器状态
ros2 topic hz /imu/data            # 应约 100 Hz

# EKF 是否正常融合 IMU
ros2 topic echo /odometry/filtered --field pose.covariance --once  # 协方差应较小
```
