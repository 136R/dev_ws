# my_bot_hw 常用命令速查表

> 适用平台：Orange Pi 5 Pro，ROS2 Humble
> 工作空间：`~/dev_ws`，包名：`my_bot_hw`

---

## 一、编译与部署

```bash
# 编译单个包（日常迭代，最快）
cd ~/dev_ws
colcon build --packages-select my_bot_hw --symlink-install

# 编译整个工作空间
colcon build --symlink-install

# source 环境（每次新终端必须执行，或写入 ~/.bashrc）
source ~/dev_ws/install/setup.bash

# 一键 source（如已写入 ~/.bashrc 则自动生效）
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc

# 查看编译日志（上次构建）
cat ~/dev_ws/log/latest_build/my_bot_hw/stdout_stderr.log
```

---

## 二、启动机器人底盘（必须先运行）

```bash
# 标准启动（串口 /dev/ttyS7，雷达 /dev/ttyUSB0）
ros2 launch my_bot_hw robot_bringup.launch.py

# 指定串口启动
ros2 launch my_bot_hw robot_bringup.launch.py serial_port:=/dev/ttyS7

# 指定雷达串口启动
ros2 launch my_bot_hw robot_bringup.launch.py lidar_port:=/dev/ttyUSB1

# 确认串口设备存在
ls -la /dev/ttyS7 /dev/ttyUSB0

# 检查串口权限（若 permission denied）
sudo chmod 666 /dev/ttyS7 /dev/ttyUSB0
# 或永久加入 dialout 组
sudo usermod -aG dialout orangepi
```

---

## 三、遥控运动

```bash
# 键盘遥控（高优先级，覆盖 Nav2 自动导航）
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard

# 发送单次速度指令（直行 0.2 m/s）
ros2 topic pub /cmd_vel_keyboard geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --once

# 发送原地旋转（0.3 rad/s，持续发送）
ros2 topic pub /cmd_vel_keyboard geometry_msgs/msg/Twist \
  "{angular: {z: 0.3}}" --rate 10

# 立即停车
ros2 topic pub /cmd_vel_keyboard geometry_msgs/msg/Twist "{}" --once
```

---

## 四、话题调试

### 4.1 查看话题列表与频率

```bash
# 列出所有话题
ros2 topic list

# 查看各关键话题频率
ros2 topic hz /odom               # 期望 ~50 Hz
ros2 topic hz /imu                # 期望 ~50 Hz
ros2 topic hz /odometry/filtered  # 期望 ~50 Hz
ros2 topic hz /scan               # 期望 ~10 Hz（C1 雷达）

# 查看话题数据（按一次 Ctrl+C 停止）
ros2 topic echo /odom --field twist.twist
ros2 topic echo /imu --field angular_velocity
ros2 topic echo /odometry/filtered --field pose.pose
```

### 4.2 里程计验证

```bash
# 查看完整 odom（含位姿 + 速度）
ros2 topic echo /odom --once

# 只看线速度与角速度
ros2 topic echo /odom --field twist.twist --once

# 实时监测 odom 位姿（直行时 pose.x 应增长）
watch -n 0.5 "ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null"
```

### 4.3 IMU 验证

```bash
# 查看 IMU 数据（含方向四元数 + 角速度）
ros2 topic echo /imu --once

# 实时打印偏航角（度）+ 角速度
python3 - << 'EOF'
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Imu

class M(Node):
    def __init__(self):
        super().__init__('m')
        self.create_subscription(Imu, '/imu', self.cb, 10)
    def cb(self, msg):
        q = msg.orientation
        yaw = math.degrees(math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)))
        print(f'yaw={yaw:7.2f}°  vyaw={msg.angular_velocity.z:6.3f} rad/s', end='\r')

rclpy.init(); rclpy.spin(M())
EOF

# 验证原地旋转时 angular_velocity.z ≈ 命令角速度
ros2 topic echo /imu --field angular_velocity
```

### 4.4 TF 链验证

```bash
# 查看当前 TF 树（生成 frames.pdf）
ros2 run tf2_tools view_frames

# 实时查看某个 TF 变换
ros2 run tf2_ros tf2_echo odom base_footprint

# 检查 map→odom→base_footprint 完整链（建图/导航时）
ros2 run tf2_ros tf2_echo map base_footprint
```

### 4.5 控制器状态

```bash
# 查看已激活的控制器列表
ros2 control list_controllers

# 查看硬件接口状态
ros2 control list_hardware_interfaces

# 查看硬件组件（插件加载状态）
ros2 control list_hardware_components
```

---

## 五、传感器调试

### 5.1 激光雷达

```bash
# 检查雷达话题（应有 /scan）
ros2 topic hz /scan
ros2 topic echo /scan --field ranges --once

# 用 RViz 可视化雷达点云（PC 端）
rviz2  # 添加 LaserScan，话题选 /scan，frame_id=laser_frame
```

### 5.2 EKF 诊断

```bash
# 查看 EKF 诊断信息
ros2 topic echo /diagnostics --once

# 对比 EKF 输入输出
ros2 topic echo /odom --field twist.twist.linear.x --once
ros2 topic echo /odometry/filtered --field twist.twist.linear.x --once
```

---

## 六、建图（Mapping）

```bash
# 终端1：启动底盘（若未启动）
ros2 launch my_bot_hw robot_bringup.launch.py

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/orangepi/dev_ws/src/my_bot_hw/config/mapper_params_hw_mapping.yaml

# 终端2：启动建图
ros2 launch my_bot_hw nav_slam.launch.py slam_mode:=mapping

# 终端3：键盘遥控建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard

# 建图完成后保存地图
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/orangepi/maps/my_real_map'}}"

# 验证地图文件生成
ls -lh ~/maps/
# 期望生成：my_real_map.data  my_real_map.yaml
```

---

cho

## 七、定位与自主导航（Localization）

```bash
# 前提：~/maps/my_real_map.data 已存在

# 终端1：启动底盘
ros2 launch my_bot_hw robot_bringup.launch.py

# 终端2：启动定位+导航
ros2 launch my_bot_hw nav_slam.launch.py slam_mode:=localization

# PC 端 RViz（通过 SSH 远程或本地）
# 1. 打开 RViz，添加 Map / LaserScan / RobotModel / Path
# 2. 用「2D Pose Estimate」点击地图上的初始位置+朝向
# 3. 用「Nav2 Goal」发送导航目标点

# 命令行发送导航目标（不用 RViz）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"

# 重置 EKF 位置估计（无需重启）
ros2 service call /ekf_filter_node/set_pose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0}}}}"
```

---

## 八、系统监控

```bash
# 查看所有运行中的节点
ros2 node list

# 查看节点详情（订阅/发布/服务）
ros2 node info /ekf_filter_node
ros2 node info /controller_manager

# 查看 ROS2 日志（实时）
ros2 topic echo /rosout | grep -E "WARN|ERROR"

# 监控 CPU/内存占用
htop

# 检查串口通信（原始字节，调试协议）
sudo stty -F /dev/ttyS7 115200 raw
sudo cat /dev/ttyS7 | xxd | head -20
```

---

## 九、硬件参数速查

| 参数         | 值                     | 说明                                       |
| ------------ | ---------------------- | ------------------------------------------ |
| 串口设备     | `/dev/ttyS7`         | STM32 (UART7-M2)                           |
| 串口波特率   | `115200`             | STM32 通信                                 |
| 雷达设备     | `/dev/ttyUSB0`       | 思岚 C1                                    |
| 雷达波特率   | `460800`             | C1 专用                                    |
| 轮间距       | `0.171 m`            | `WHEEL_TREAD_1`（来自 Config.h）         |
| 轮半径       | `0.0325 m`           | `WHEEL_DIAMETER_1/2`（来自 Config.h）    |
| 减速比       | `20:1`               | `RATIO_1`                                |
| 最大线速度   | `≈ 0.82 m/s`        | `MOTOR_MAX_RPM_1/60/RATIO_1×π×D×0.8` |
| 地图保存路径 | `~/maps/my_real_map` | `mapper_params_hw.yaml`                  |
| 地图加载路径 | `~/maps/my_real_map` | 无需 `.data` 扩展名                      |

---

## 十、快速故障排查

| 现象             | 排查命令                                                                      |
| ---------------- | ----------------------------------------------------------------------------- |
| 控制器未激活     | `ros2 control list_controllers`                                             |
| `/odom` 无数据 | `ros2 topic hz /odom`，检查串口权限                                         |
| `/scan` 无数据 | `ls /dev/ttyUSB*`，检查雷达波特率 460800                                    |
| TF 断链          | `ros2 run tf2_tools view_frames`                                            |
| EKF 无输出       | `ros2 topic hz /odometry/filtered`；检查 `/imu` 和 `/odom` 是否有数据   |
| 导航不启动       | `ros2 node list \| grep nav2`；确认地图文件 `~/maps/my_real_map.data` 存在 |
| 建图漂移         | 检查 `wheel_separation` 是否为 `0.171`，执行轮距校准旋转测试              |
