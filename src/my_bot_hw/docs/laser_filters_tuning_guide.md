# 激光雷达过滤器调试指南

> 配置文件：`config/laser_filters.yaml`
> 过滤链：`/scan`（原始）→ **laser_filter_node** → `/scan_filtered`（干净）
> SLAM / Nav2 均订阅 `/scan_filtered`

---

## 一、整体过滤链与顺序

```
/scan (原始)
  │
  ├─ [1] LaserScanRangeFilter         距离范围裁剪（去盲区 + 去远点）
  ├─ [2] LaserScanSpeckleFilter       斑点过滤（去孤立噪点）
  └─ [3] LaserScanAngularBoundsFilter 角度屏蔽（遮挡扇区，预留位）
  │
/scan_filtered (干净)
```

**顺序说明**：先做距离裁剪再做斑点检测，可避免无效远点被 Speckle 误判为"邻居点"干扰窗口计算。

---

## 二、LaserScanRangeFilter（距离范围过滤）

### 作用
将距离不在 `[lower_threshold, upper_threshold]` 范围内的点替换为 `±inf`，SLAM 和 Nav2 自动忽略 `inf` 值。

### 参数说明

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `lower_threshold` | `0.10` | 最小有效距离（米）。低于此值视为盲区噪点 |
| `upper_threshold` | `5.0` | 最大有效距离（米）。高于此值视为无效远点 |
| `lower_replacement_value` | `-.inf` | 近点替换值（`-.inf` = 清除） |
| `upper_replacement_value` | `.inf` | 远点替换值（`.inf` = 清除） |
| `use_message_range_limits` | `false` | 是否使用消息头里的 range_min/max，设为 false 用上方手动值 |

### 调大 / 调小的物理效果

| 参数 | 调大 | 调小 |
|------|------|------|
| `lower_threshold` | 去掉更多近点，机器人周围空白区变大，可能丢失近处障碍物 | 保留更多近点，盲区噪点可能污染 costmap |
| `upper_threshold` | 保留更远的点，建图范围更大，但远处噪点也更多 | 只使用可靠近距离，建图更干净但看不到远处墙 |

### 调试方法
1. 启动机器人，打开 RViz，添加 LaserScan 分别显示 `/scan` 和 `/scan_filtered`
2. 将机器人放在已知 0.08m 的近物旁，观察 `/scan` 的近点是否被 `/scan_filtered` 过滤
3. `lower_threshold` 设置原则：**比 C1 盲区上边界（约 0.08m）多留 0.02m 安全余量 = 0.10m**

```bash
# 查看当前 scan 的距离范围
ros2 topic echo /scan --field range_min --once   # C1 输出约 0.08
ros2 topic echo /scan --field range_max --once   # C1 输出约 12.0
```

---

## 三、LaserScanSpeckleFilter（斑点过滤器）

### 作用
检测并删除"孤立点"——与周围点距离差异过大的单个或少数点。典型场景：玻璃反射点、粉尘颗粒、电缆截面点。

### 参数说明

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `filter_type` | `0` | `0` = DISTANCE 模式（距离差判断），`1` = EUCLIDEAN 模式（欧式聚类） |
| `max_range` | `2.0` | 仅对此距离以内的点做斑点检测（米）。远处点不参与，节省计算 |
| `max_range_difference` | `0.1` | 窗口内相邻点的最大允许距离差（米）。超过此值视为斑点 |
| `filter_window` | `2` | 检测窗口半径（检查前后各 2 个点，共 5 点窗口）|

### filter_type 选择

| 模式 | 适用场景 | 特点 |
|------|---------|------|
| `0` (DISTANCE) | 普通室内环境，障碍物边缘清晰 | 速度快，参数直观（距离差阈值） |
| `1` (EUCLIDEAN) | 障碍物形状复杂、大量孤立点 | 效果更强，计算量稍大 |

### 调大 / 调小的物理效果

| 参数 | 调大 | 调小 |
|------|------|------|
| `max_range` | 对更远处的点也做斑点检测，过滤更彻底 | 只过滤近处，远处噪点保留 |
| `max_range_difference` | 允许更大距离跳变才算斑点，过滤更宽松 | 更灵敏，容易误删障碍物边缘点 |
| `filter_window` | 窗口更大，判断更稳健，但可能连带删除正常点 | 窗口更小，只检测紧邻点，误删少 |

### 调试方法
1. 在玻璃墙或金属柜旁观察 `/scan` 是否有明显的离群跳点
2. 逐步减小 `max_range_difference`（如 0.2 → 0.1 → 0.05）观察离群点消失情况
3. 注意不要设得太小（< 0.05）——薄门框、桌腿边缘本身就有距离跳变，会被误删

```bash
# 对比过滤前后点数（差值越大说明过滤效果越明显）
ros2 topic echo /scan          --once | python3 -c "import sys; d=sys.stdin.read(); print('raw nan:', d.count('nan'))"
ros2 topic echo /scan_filtered --once | python3 -c "import sys; d=sys.stdin.read(); print('filtered nan:', d.count('nan'))"
```

---

## 四、LaserScanAngularBoundsFilterInPlace（角度边界过滤）

### 作用
将指定角度扇区内的点全部设为 `nan`，用于屏蔽机器人自身遮挡（支撑柱、线缆、传感器外壳）造成的固定噪点。

**InPlace 版本**：保留原有点数组大小，被屏蔽角度替换为 `nan`（非 InPlace 版本会删除点改变数组长度，可能导致 SLAM 角度映射错误）。

### 参数说明

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `lower_angle` | `0.0` | 【预留位】屏蔽扇区起始角（弧度）|
| `upper_angle` | `0.0` | 【预留位】屏蔽扇区结束角（弧度）|

**当前为占位状态**：`lower_angle == upper_angle == 0.0`，不过滤任何角度。

### 角度方向约定
- ROS2 激光雷达坐标系：**正前方 = 0 rad，逆时针为正**
- `lower_angle < upper_angle`：屏蔽从 lower 到 upper 的逆时针扇区
- 示例：屏蔽正后方 ±15°（车体支撑柱）→ `lower_angle: -0.26, upper_angle: 0.26`（注意此时需要用两段分别处理前后，或旋转坐标系）

### 调试方法（找遮挡角度）

```bash
# Step 1：静止放置机器人，观察 /scan 中的固定噪点
# 在 RViz 中显示 /scan，记录总是出现短距离点的方向

# Step 2：用 Python 找出哪些角度有异常近点
python3 - << 'EOF'
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
    def cb(self, msg):
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if 0.05 < r < 0.25:   # 可疑近点范围（米）
                print(f'疑似遮挡：angle={math.degrees(angle):6.1f}°  range={r:.3f}m')

rclpy.init(); rclpy.spin(ScanAnalyzer())
EOF

# Step 3：确认角度范围后填写到 laser_filters.yaml
# 例如：支撑柱在 170° 方向，宽度约 20°
#   lower_angle: 2.79   # 170° - 10° = 160° = 2.79 rad
#   upper_angle: 3.14   # 170° + 10° = 180° = 3.14 rad
```

### 多段遮挡（两根支撑柱）
如需屏蔽两个不连续的角度区间，在 `scan_filter_chain` 中**添加第二个同类过滤器**即可：

```yaml
  - name: angular_bounds_filter_2
    type: laser_filters/LaserScanAngularBoundsFilterInPlace
    params:
      lower_angle: X.XX   # 第二段遮挡起始角
      upper_angle: X.XX   # 第二段遮挡结束角
```

---

## 五、整体调试流程

```
Step 1  先只启用 RangeFilter，验证近点/远点裁剪效果
        → ros2 topic hz /scan_filtered 确认话题存在
        → RViz 对比 /scan 和 /scan_filtered

Step 2  加入 SpeckleFilter，观察玻璃/金属噪点变化
        → 调整 max_range_difference 直到孤立点消失但边缘不被误删

Step 3  运行 scan_analyzer 脚本找出固定遮挡角度
        → 填写 AngularBoundsFilter 参数

Step 4  跑一圈建图，对比加滤波前后的地图质量
        → 墙体线条是否更连续，孤立障碍物标记是否减少
```

---

## 六、验证命令速查

```bash
# 确认 laser_filter_node 已启动
ros2 node list | grep laser_filter

# 确认 /scan_filtered 话题存在且有数据
ros2 topic hz /scan_filtered        # 应与 /scan 频率相同（约 10 Hz）
ros2 topic echo /scan_filtered --field header.stamp --once

# 统计过滤掉的点数（nan 数量）
ros2 topic echo /scan          --once 2>/dev/null | grep -c "nan"
ros2 topic echo /scan_filtered --once 2>/dev/null | grep -c "nan"

# RViz 可视化对比（同时添加两个 LaserScan 话题，设置不同颜色）
# /scan          → 红色（原始，噪点多）
# /scan_filtered → 绿色（过滤后，用于 SLAM/Nav2）
```
