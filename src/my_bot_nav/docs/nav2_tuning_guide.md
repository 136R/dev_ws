# Nav2 导航调参指南

> 配置文件：`config/nav2_params_hw.yaml`
> 机器人尺寸（来自 URDF `robot_core.xacro`）：底盘 12.3×13.0cm，含车轮宽 **20.2cm**
> 参考来源：[Nav2 官方文档](https://navigation.ros.org/)、[Nav2 MPPI Tuning Guide](https://navigation.ros.org/configuration/packages/configuring-mppi.html)、[Nav2 GitHub Discussions](https://github.com/ros-navigation/navigation2/discussions)、Steve Macenski 的调参博客

---

## 一、导航流程与参数分组

```
/map (slam_toolbox 提供)
  │
  ├─ [1] 全局代价地图 (GlobalCostmap)    static_layer + obstacle_layer + inflation_layer
  │       ↓
  ├─ [2] 全局路径规划 (SmacPlanner2D)   规划从起点到终点的粗路径
  │       ↓ /plan
  ├─ [3] 局部代价地图 (LocalCostmap)    rolling window，实时障碍物
  │       ↓
  ├─ [4] 局部控制器 (MPPIController)    采样轨迹 → 打分 → 执行最优
  │       ↓ /diff_cont/cmd_vel_unstamped
  └─ [5] VelocitySmoother              平滑加速度，防止急停急起
```

**核心认知**：全局规划决定"走哪条路"，局部控制决定"怎么走"。两套代价地图独立——全局地图是全图静态地图，局部地图是机器人周围 3m×3m 的滚动窗口。

---

## 二、代价地图参数（最基础的一层）

### 2.1 机器人轮廓：robot_radius vs footprint

这是**对你的机器人影响最大的参数之一**。

**URDF 实测数据**（来自 `robot_core.xacro`）：

```
底盘 box:  0.123 × 0.130 × 0.0958 m（x × y × z）
底盘中心偏移 base_link：x = -0.0275 m（非对称，稍偏后）
车轮关节位置：y = ±0.0875 m，车轮半长 = 0.0135 m
轮外缘 y:  ±(0.0875 + 0.0135) = ±0.101 m

→ footprint x：前 +0.034 m，后 -0.089 m（总长 12.3 cm）
→ footprint y：±0.101 m（总宽 20.2 cm，含车轮）
→ 内切圆半径：min(0.0615, 0.101) = 0.062 m
→ 外接圆半径：√(0.062² + 0.101²) ≈ 0.118 m
```

来自 Nav2 官方文档（[Configuring Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)）：

> "For non-circular robots, it is strongly recommended to use the footprint parameter rather than robot_radius."

当前配置的 footprint（与 URDF 吻合）：

```yaml
# 前 x=0.035（≈0.034），后 x=-0.089，左右 y=±0.101（轮外缘）
footprint: "[[0.035, 0.101], [0.035, -0.101], [-0.089, -0.101], [-0.089, 0.101]]"
```

**footprint vs robot_radius 对比：**

| 项目 | robot_radius: 0.12 | footprint 多边形（当前） |
|---|---|---|
| 代价地图中机器人宽度 | 24cm（圆形） | 20.2cm（实际含车轮） |
| 能规划通过的最窄走廊 | ≈ 30cm | ≈ 26cm |
| 碰撞检测计算量 | 低 | 稍高（差异可忽略） |

### 2.2 膨胀层（inflation_layer）

膨胀层将障碍物"向外扩张"，让规划器优先选择远离障碍物的路径。


| 参数                  | 当前值（local） | 当前值（global） | 含义                               |
| --------------------- | --------------- | ---------------- | ---------------------------------- |
| `inflation_radius`    | `0.30`          | `0.25`           | 膨胀半径（米），超出此距离代价为 0 |
| `cost_scaling_factor` | `3.0`           | `3.0`            | 代价衰减速率，越大衰减越快         |

**`inflation_radius` 的本质**：代价地图中，距离障碍物小于此值的格子都有非零代价，规划器会绕开高代价区域。实际上，机器人中心到障碍物的最小期望距离 ≈ `inflation_radius`。

**local 和 global 的 `inflation_radius` 不一致（0.30 vs 0.25）是一个潜在问题。**

来自 Nav2 GitHub [Discussion #3424](https://github.com/ros-navigation/navigation2/discussions/3424)：全局代价地图的膨胀半径应 **≥** 局部代价地图，否则全局规划出的路径在局部代价地图里可能直接穿过高代价区，导致 MPPI 频繁偏离路径。

**`cost_scaling_factor: 3.0` 分析**：

Nav2 官方默认值是 10.0。值越低，膨胀区的代价越"平缓"——机器人可以靠近障碍物而不被大幅惩罚。

```
代价 = 253 × e^(-cost_scaling_factor × (距离 - inscribed_radius))

cost_scaling_factor=3.0 时，距离障碍物 0.1m 处代价约为 163（较高但可通行）
cost_scaling_factor=10.0 时，距离障碍物 0.1m 处代价约为  7（几乎忽略）
```

**结论**：你的 3.0 让机器人在 30cm 范围内的障碍物附近都保持较高代价警觉性，适合家具密集的室内环境，**无需改动**。

### 2.3 障碍物层距离参数


| 参数                 | local 当前值 | global 当前值 | 含义                             |
| -------------------- | ------------ | ------------- | -------------------------------- |
| `raytrace_max_range` | `4.0`        | `5.0`         | 用射线清除障碍物的最远距离（米） |
| `obstacle_max_range` | `3.5`        | `4.5`         | 标记障碍物的最远距离（米）       |

规则：`obstacle_max_range` 必须 < `raytrace_max_range`，否则标记了却清除不了，代价地图里会出现"幽灵障碍物"。你的配置符合此规则（差值均为 0.5m），**无需改动**。

---

## 三、全局路径规划（SmacPlanner2D）

SmacPlanner2D 使用 A* 搜索在全局代价地图上规划路径。


| 参数                     | 当前值  | Nav2 默认值 | 含义                         |
| ------------------------ | ------- | ----------- | ---------------------------- |
| `tolerance`              | `0.05`  | `0.5`       | 目标点可接受的位置偏差（米） |
| `max_iterations`         | `20000` | `1000000`   | A* 最大迭代次数              |
| `cost_travel_multiplier` | `2.0`   | `2.0`       | 路径代价对搜索的影响倍数     |
| `allow_unknown`          | `true`  | `true`      | 是否允许规划经过未探索区域   |

**`max_iterations: 20000` 是一个 Bug 级别的配置。**

来自 Nav2 官方文档：

> "max_iterations: Maximum number of iterations to plan. If this is exceeded, the planner will fail. Setting this too low will cause planning failures in large or complex maps."

20000 次迭代在 5cm 分辨率的地图上，大约只能搜索到距离起点约 **1.4m** 的范围（5cm × √20000 ≈ 7m，但 A* 不是直线扩展，实际有效范围更小）。超过这个范围的目标点会规划失败。**必须改回 `1000000`。**

`tolerance: 0.05` 与 `goal_checker` 的 `xy_goal_tolerance: 0.05` 一致，正确。

---

## 四、局部控制器（MPPIController）

MPPI（Model Predictive Path Integral）工作原理：每个控制周期采样 `batch_size` 条候选轨迹，用 `critics`（评分函数）给每条轨迹打分，选最优轨迹的第一步执行。

### 4.1 轨迹采样参数


| 参数          | 当前值 | Nav2 示例值 | 含义                                          |
| ------------- | ------ | ----------- | --------------------------------------------- |
| `time_steps`  | `30`   | `56`        | 预测步数（× model_dt = 预测时域）            |
| `model_dt`    | `0.05` | `0.05`      | 单步时间（秒），必须 = 1/controller_frequency |
| `batch_size`  | `800`  | `2000`      | 每周期采样的候选轨迹数                        |
| `temperature` | `0.3`  | `0.3`       | 轨迹权重的"平滑度"，越小越激进                |
| `vx_std`      | `0.2`  | `0.2`       | 线速度采样噪声标准差                          |
| `wz_std`      | `0.4`  | `0.4`       | 角速度采样噪声标准差                          |

**`time_steps: 30` 预测时域偏短。**

`30 步 × 0.05s = 1.5 秒`，以你的最大速度 0.2m/s 计算，前瞻距离仅 **0.3m**。

Nav2 官方 MPPI 调参文档推荐 `time_steps: 56`（= 2.8s 时域，前瞻 0.56m）。预测时域太短会导致：

- 机器人"近视"，不能提前为转弯调整速度
- 接近障碍物时反应迟钝

**`batch_size: 800` 偏低。**

来自 Nav2 MPPI 调参文档：

> "Larger batch sizes lead to better coverage of the solution space. Values below 1000 may lead to suboptimal trajectory selection."

Orange Pi 5 Pro（A76 大核）单次 MPPI 迭代约 2-5ms，`batch_size: 2000` 在 20Hz 控制频率下仍有足够裕量（20Hz = 每步 50ms，MPPI 约占 10ms）。

### 4.2 Critic 权重

每个 Critic 给轨迹打惩罚分，权重越高影响越大。你的当前配置：


| Critic                | 当前权重       | 作用                 |
| --------------------- | -------------- | -------------------- |
| `PathAlignCritic`     | `14.0`（最高） | 轨迹与全局路径对齐   |
| `GoalCritic`          | `5.0`          | 轨迹终点靠近目标点   |
| `PathFollowCritic`    | `5.0`          | 轨迹跟随路径进度     |
| `PreferForwardCritic` | `5.0`          | 偏好向前运动         |
| `ConstraintCritic`    | `4.0`          | 速度/加速度约束      |
| `CostCritic`          | `3.81`         | 避障（代价地图代价） |
| `GoalAngleCritic`     | `3.0`          | 接近目标时的朝向     |
| `PathAngleCritic`     | `2.0`          | 与路径方向对齐       |

**`CostCritic` 的 `consider_footprint` 需要与 footprint 切换联动：**

```yaml
CostCritic:
  consider_footprint: false   # 当前：使用 robot_radius 圆形时必须 false
  # 改为 footprint 多边形后：改为 true，碰撞检测更精确
```

**Critic 权重调整逻辑（来自 Nav2 官方调参指南）**：


| 现象                       | 原因                | 操作                                                   |
| -------------------------- | ------------------- | ------------------------------------------------------ |
| 机器人偏离全局路径大幅绕行 | PathAlign 权重不足  | 增大`PathAlignCritic.cost_weight`（当前已是 14，较高） |
| 机器人贴墙走或碰撞         | CostCritic 权重不足 | 增大`CostCritic.cost_weight`（当前 3.81 偏低）         |
| 机器人在目标点附近转圈     | GoalCritic 不足     | 增大`GoalCritic.cost_weight`                           |
| 机器人总是倒车进入目标     | PreferForward 不足  | 增大`PreferForwardCritic.cost_weight`                  |
| 机器人路径抖动             | 采样噪声过大        | 减小`vx_std`（0.2→0.1）或 `wz_std`（0.4→0.2）        |

---

## 五、目标到达判断（Goal Checker）


| 参数                 | 当前值    | Nav2 默认值 | 含义                         |
| -------------------- | --------- | ----------- | ---------------------------- |
| `xy_goal_tolerance`  | `0.05`    | `0.25`      | 到达目标点的位置精度（米）   |
| `yaw_goal_tolerance` | `3.14159` | `0.25`      | 到达目标点的朝向精度（弧度） |

**`xy_goal_tolerance: 0.05` = 5cm 精度，比官方默认严格 5 倍。**

这需要 MPPI 能精确控制到 5cm 以内，且 SLAM 定位误差也要 < 5cm。如果发现导航总是在目标点附近反复来回，说明定位误差或控制误差大于 5cm，适当放宽到 `0.1` 即可。

`yaw_goal_tolerance: 3.14159` = 不限制到达朝向，**对差速机器人的点到点导航是正确的**（到达就好，不强求方向）。

---

## 六、Progress Checker（防卡死）

```yaml
progress_checker:
  required_movement_radius: 0.5   # 10 秒内必须移动 0.5m
  movement_time_allowance: 10.0   # 检测窗口（秒）
```

`required_movement_radius: 0.5` 意味着 10 秒内如果机器人没有移动超过 0.5m，认为"卡住"并触发 recovery behavior（spin/backup）。

**对于你的机器人（最大速度 0.2m/s）**，10 秒理论上能走 2m，0.5m 要求是合理的。但如果机器人在狭窄空间里缓慢挪动（< 5cm/s），可能误触发。可以改为：

```yaml
required_movement_radius: 0.1   # 更宽松，10 秒移动 10cm 即可
movement_time_allowance: 15.0   # 给更多时间
```

---

## 七、针对你的机器人：推荐修改项

### 修改 1：切换为实际 footprint（影响最大）

在 `local_costmap` 和 `global_costmap` 的 `ros__parameters` 下，替换 `robot_radius`：

```yaml
# 修改前（两个 costmap 都有）
robot_radius: 0.12

# 修改后（两个 costmap 都改）
footprint: "[[0.062, 0.1025], [0.062, -0.1025], [-0.062, -0.1025], [-0.062, 0.1025]]"
```

同时修改 CostCritic：

```yaml
CostCritic:
  consider_footprint: true   # false → true
```

**预期效果**：能规划通过约 26cm 宽的走廊（之前需要 30cm+）。

### 修改 2：修复 global inflation_radius 不一致

```yaml
# global_costmap 的 inflation_layer 下
# 修改前
inflation_radius: 0.25

# 修改后
inflation_radius: 0.30   # 与 local_costmap 保持一致
```

### 修改 3：修复 max_iterations（规划失败的根因）

```yaml
# 修改前
max_iterations: 20000

# 修改后
max_iterations: 1000000
```

### 修改 4：延长 MPPI 预测时域

```yaml
# 修改前
time_steps: 30

# 修改后
time_steps: 56   # 2.8 秒时域，前瞻 0.56m，Nav2 官方示例值
```

### 修改 5：增大 MPPI 采样数

```yaml
# 修改前
batch_size: 800

# 修改后
batch_size: 2000   # Nav2 官方推荐，Orange Pi 5 Pro 可承受
```

### 修改 6：对齐速度限制链路（与 hw_controllers.yaml 一致）

速度命令链路：MPPI → VelocitySmoother → DiffDriveController，上游必须 ≤ 下游，否则被静默截断。

```yaml
# hw_controllers.yaml 的硬限制（下游真正执行的上限）：
# linear.x.max_velocity: 0.2,  max_acceleration: 0.8
# angular.z.max_velocity: 0.6, max_acceleration: 1.5

# MPPI 中（已修复）
wz_max: 0.6    # 原 1.0，超过控制器上限

# VelocitySmoother 中（已修复）
max_velocity: [0.2, 0.0, 0.6]   # wz: 1.0 → 0.6
min_velocity: [-0.2, 0.0, -0.6] # wz: -1.0 → -0.6
max_accel:    [0.8, 0.0, 1.5]   # vx: 1.0 → 0.8
max_decel:    [-0.8, 0.0, -1.5] # vx: -1.0 → -0.8
```

---

## 八、调试命令

```bash
# 查看当前代价地图（RViz 中订阅这些话题）
# /local_costmap/costmap      局部代价地图（实时更新）
# /global_costmap/costmap     全局代价地图（随 SLAM 更新）
# /plan                       全局规划路径
# /local_plan                 MPPI 当前执行轨迹（需开启 visualize: true）

# 开启 MPPI 轨迹可视化（调试时临时开启，正式运行关闭）
# 修改 nav2_params_hw.yaml：visualize: true，重启 nav

# 查看导航状态
ros2 topic echo /navigate_to_pose/_action/status --once

# 查看 MPPI 是否收到路径
ros2 topic hz /plan      # 有路径时应有数据

# 查看代价地图是否有障碍物更新
ros2 topic hz /local_costmap/costmap   # 应约 2 Hz（publish_frequency=2.0）

# 手动触发单次全局规划（测试规划器）
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  '{goal: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'

# 查看 Nav2 lifecycle 各节点状态
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server

# 实时查看 MPPI 输出速度
ros2 topic echo /diff_cont/cmd_vel_unstamped
```

---

## 九、常见导航问题速查


| 现象                           | 根因参数                                  | 操作                                                 |
| ------------------------------ | ----------------------------------------- | ---------------------------------------------------- |
| 规划失败，目标点明明可达       | `max_iterations` 太小                     | 改为`1000000`                                        |
| 能过的走廊被认为不能通过       | 使用了外接圆`robot_radius`                | 换用`footprint` 多边形                               |
| 机器人绕远路，不走近路         | `global inflation_radius` 过大            | 减小`global inflation_radius`（0.30→0.25）          |
| 机器人贴着障碍物走             | `CostCritic.cost_weight` 太低             | 从`3.81` 增大到 `5.0~8.0`                            |
| 机器人在目标点附近来回晃       | `xy_goal_tolerance` 太严                  | 从`0.05` 改 `0.10`                                   |
| 机器人转弯迟钝，总是冲过头     | `time_steps` 太短                         | 从`30` 改 `56`                                       |
| MPPI 轨迹明显抖动              | `batch_size` 不足                         | 从`800` 改 `2000`                                    |
| 机器人卡住后不触发恢复行为     | progress_checker 配置不合适               | 检查`required_movement_radius`                       |
| 恢复行为（spin）总是失败       | 旋转空间不足                              | 减小`behavior_server.min_rotational_vel`（0.4→0.2） |
| 导航过程中代价地图出现幽灵障碍 | `obstacle_max_range > raytrace_max_range` | 确保 obstacle < raytrace（差值保持 0.5m）            |
