# Nav2 调参优先级与验收方案

基于机器人参数（底盘 124×202 mm，最高速 0.2 m/s，DWB + SmacPlanner2D），以下是社区公认的调参顺序（参考 Nav2 官方 Tuning Guide、Steve Macenski 博客及 tb3/turtlebot 实践）。

---

## 第一阶段：Costmap（代价地图）

**原则：先把障碍物膨胀做对，后续所有规划/控制才有意义。**

### 1.1 Footprint 验证（必调前置）

当前 footprint（宽 202 mm，长 124 mm）：

```
[[0.035, 0.101], [0.035, -0.101], [-0.089, -0.101], [-0.089, 0.101]]
```

验收方式：在 RViz2 中订阅 `/local_costmap/published_footprint`，确认机器人轮廓在矩形范围内。

---

### 1.2 inflation_radius — 膨胀半径

障碍物周围的涨价圈半径，决定机器人与障碍物的最小安全距离。
计算依据：≥ footprint 半宽（0.101 m）+ 安全余量（0.05～0.15 m）。

- 太小：机器人沿墙贴边行走、易碰墙
- 太大：窄道无法通行，卡在门口

> `local_costmap` 和 `global_costmap` 都要改；`global` 通常比 `local` 大 0.05 m。

建议值：

```yaml
# global_costmap
inflation_radius: 0.35

# local_costmap
inflation_radius: 0.20
```

---

### 1.3 cost_scaling_factor — 膨胀梯度陡峭程度

控制代价从障碍边缘向外衰减的速率。值越大衰减越快（梯度越陡）。

- 太小（< 1.5）：全区域高 cost，规划器总绕远路
- 太大（> 5.0）：梯度太陡，机器人贴近障碍边缘也认为安全

> `local_costmap` 和 `global_costmap` 一般取相同值；若窄通道无法通过，可对 `local_costmap` 单独加大 0.5。

建议值：

```yaml
cost_scaling_factor: 3.0
```

**验收标准（Costmap 阶段通关）：**

```
RViz2 打开 local_costmap，目视检查：
✓ 障碍物周围有颜色渐变圈（蓝→红）
✓ 圈的半径目测约 20～35 cm
✓ 走廊中央 cost 颜色比两侧浅
```

---

## 第二阶段：Global Planner（SmacPlanner2D）

只需验证规划路径"合理"，不需要完美。

### 2.1 tolerance — 终点容忍半径

全局规划器认为"到达终点"的距离阈值，一般与 `goal_checker` 的 `xy_goal_tolerance` 保持一致。

- 太小：规划器认为无法到达终点，频繁规划失败
- 太大：路径终点偏离目标

> 若全局路径在接近目标点时出现阶梯状折线，导致局部规划区持续卡住，可尝试改为 0.01。

建议值：

```yaml
tolerance: 0.05
```

---

### 2.2 cost_travel_multiplier — 规划器绕障积极性

控制全局规划器对高代价区域的回避程度。

- 增大：路径更绕道，远离障碍
- 减小：路径更短，但接近障碍

当前值 2.0 合理，暂无需修改。

---

### 2.3 expected_planner_frequency — 全局规划频率

SmacPlanner2D 全局规划在嵌入式平台耗时较长，标准做法是 1～5 Hz。

- 太高：规划器实际达不到设定频率，持续输出警告日志
- 太低：路径更新不及时，响应变慢

建议值：

```yaml
expected_planner_frequency: 1.0
```

---

### 2.4 调试全局规划路径（命令行验证）

单独测试规划器输出，无需启动完整 Nav2 导航：

```bash
# 向右上方规划（y=+0.5，用于验证正常侧）
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"

# 向右下方规划（y=-0.5，用于验证对称性）
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```


| 字段                    | 正常                         | 异常                                   |
| ----------------------- | ---------------------------- | -------------------------------------- |
| planning_time           | < 100 ms                     | > 1 s：地图复杂或参数异常              |
| 路径点数                | 与距离成比例（约 0.05 m/点） | 点数极多：路径绕远                     |
| 末段 orientation z 跳变 | 相邻点 < 30°                | 末段突变 > 60°：smoother 过度平滑     |
| 左右目标路径对比        | 曲率对称                     | 一侧急转：障碍分布不对称，需查代价地图 |

**SmacPlanner2D 平滑器参数（影响路径形状，非 inflation）：**

```yaml
# planner_server → GridBased → smoother
w_smooth: 0.3                        # 平滑权重（越高曲线越弯，但可能切入膨胀层）
w_data: 0.3                          # 保真权重（越高越贴近原始栅格路径，越直）
cost_travel_multiplier: 3.0          # 规划器绕障积极性
use_final_approach_orientation: true # 在 A* 阶段约束进入角度，消除末段急转
```

> **注意：** `inflation_radius` / `cost_scaling_factor` 影响路径走哪条走廊（routing），
> 不影响路径曲线形状（shape）。形状由 `w_smooth` / `w_data` 决定。

---

## 第三阶段：local_costmap + DWB Local Planner（最核心，调参最多）

Nav2 官方对 DWB 的定性：最难调但最可控。局部规划的调参范围 = `local_costmap`（DWB 的感知输入）+ `controller_server`（DWB 的决策逻辑），与全局规划的 `global_costmap` + `planner_server` 完全对称。

---

### 3.1 local_costmap — inflation_radius — 局部膨胀半径

机器人半宽 101 mm，当前膨胀半径 120 mm，安全余量仅 19 mm。
DWB 候选轨迹贴着障碍边缘也能通过 BaseObstacle 评分，等于避障失效。

- 太小：安全余量不足，DWB 允许轨迹贴墙通过
- 太大：局部与全局代价不一致，DWB 跟不上全局路径

> `local` 偏中（0.20～0.25），`global` 偏大（0.25～0.35），但不宜差距过大。

建议值：

```yaml
inflation_layer:
  cost_scaling_factor: 3.0
  inflation_radius: 0.20
```

---

### 3.2 local_costmap — update_frequency — 局部地图更新频率

局部代价地图的刷新率，决定动态障碍物被纳入避障决策的响应速度。
DWB 每次评分都基于当前地图快照，更新越慢，障碍物位置越滞后。

- 太低（< 5 Hz）：动态障碍物更新滞后，DWB 基于过期地图决策，动态避障失效
- 太高（> 20 Hz）：CPU 占用显著上升，在嵌入式平台上可能拖慢整体控制频率

当前值 20.0 Hz 合理，暂无需修改。✓

---

### 3.3 速度/加速度参数

DWB 速度参数与 `hw_controllers.yaml` 已对齐（0.2 m/s，0.6 rad/s）。✓

---

### 3.4 sim_time — 局部轨迹前瞻时间

DWB 向前模拟候选轨迹的时间长度，决定机器人"看多远"来做决策。
公式参考：`sim_time ≈ 速度/加速度 × 2 = 0.2/0.8 × 2 = 0.5`，但实际经验值更高。

- 太短（< 1.0 s）：前瞻太近，遇障急转，易震荡
- 太长（> 3.0 s）：前瞻太远，低速机器人计算量爆炸

当前值 1.7 s 在推荐范围（1.5～2.0 s）内。✓

---

### 3.5 DWB Critics 权重 — 各行为优先级

每个 Critic 的 scale 值越大，该行为在轨迹评分中越优先。


| Critic             | 推荐范围 | 作用               |
| ------------------ | -------- | ------------------ |
| BaseObstacle.scale | 10～50   | 避障权重           |
| PathAlign.scale    | 16～64   | 跟路径方向对齐     |
| PathDist.scale     | 16～64   | 跟路径横向距离     |
| GoalAlign.scale    | 8～32    | 朝目标方向对齐     |
| GoalDist.scale     | 8～32    | 朝目标距离减小     |
| RotateToGoal.scale | 16～64   | 到达终点时原地转向 |

调参方向：`PathAlign > PathDist` 时跟路径方向好；`PathDist > PathAlign` 时跟路径横向距离好。

**当前 `BaseObstacle.scale: 0.1` 极其危险 —— 几乎不惩罚障碍物穿越，机器人会撞墙。**

建议值：

```yaml
BaseObstacle.scale: 20.0
PathAlign.scale: 32.0
PathDist.scale: 32.0
GoalAlign.scale: 24.0
GoalDist.scale: 24.0
RotateToGoal.scale: 32.0
```

---

### 3.6 vx_samples / vtheta_samples — 速度采样数量

DWB 在速度空间中均匀采样候选轨迹，采样数越多决策越精细但计算量越大。

- 太少：候选轨迹稀疏，容易错过最优解
- 太多：CPU 占用高，控制频率下降

当前 `vx_samples: 20`，`vtheta_samples: 20` 对 OPi5 Pro 性能合理。若 CPU 占用高可降至 10。

---

### 3.7 min_y_velocity_threshold — Y 向速度有效阈值

差速轮 Y 方向速度恒为 0，此阈值用于过滤 Y 向微小漂移噪声。

- 太大：把正常的 Y 向零漂误判为有效运动，影响轨迹评分

建议值：

```yaml
min_y_velocity_threshold: 0.001
```

---

### 3.8 PathAlign.forward_point_distance — 朝向采样前向距离

PathAlign Critic 在当前位置前方取样一个点，用该点的方向评估轨迹朝向对齐程度。

- 太小（< 0.15 m）：差速车低速大转弯时朝向对齐效果差，易横向漂移
- 太大：对直线路径过于敏感，转弯响应迟钝

建议值：

```yaml
PathAlign.forward_point_distance: 0.2
```

---

### 3.9 xy_goal_tolerance — 终点位置容忍距离

该参数在两处出现，含义不同，需分别对待：


| 位置                                       | 作用                   | 建议值                           |
| ------------------------------------------ | ---------------------- | -------------------------------- |
| `general_goal_checker.xy_goal_tolerance`   | 判定导航任务完成       | 0.15（防里程计漂移导致终点抖动） |
| `FollowPath.xy_goal_tolerance`（DWB 内部） | 触发终点减速模式的距离 | 保持 0.05 不变                   |

建议值：

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.15
# FollowPath.xy_goal_tolerance 保持 0.05 不变
```

---

### 3.10 DWB 验证方法

DWB 是响应式控制器，没有类似 `compute_path_to_pose` 的离线测试接口，必须在导航运行中验证。

#### 第一步：静态检查（不启动导航）

```bash
# 确认参数已正确加载
ros2 param dump /controller_server

# 实时修改单个参数，无需重启 nav2
ros2 param set /controller_server FollowPath.BaseObstacle.scale 20.0
ros2 param set /controller_server FollowPath.sim_time 1.7
ros2 param set /controller_server FollowPath.PathDist.scale 32.0
```

#### 第二步：发送近距离目标触发 DWB

```bash
# 近处目标，便于隔离观察 DWB 行为（不受长距离路径干扰）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

#### 第三步：监控关键话题

```bash
ros2 topic echo /diff_cont/cmd_vel_unstamped   # 速度指令输出
ros2 topic echo /local_plan                    # DWB 选出的最优局部路径
# RViz2 订阅 /local_costmap/costmap            # 局部代价地图
```

#### 第四步：RViz2 轨迹扇形可视化

已开启 `debug_trajectory_details: True`，在 RViz2 中添加以下话题：

```
/dwb_controller_server/trajectories   ← 全部候选轨迹（彩色扇形）
/local_plan                           ← 最优轨迹（绿线）
/local_costmap/costmap                ← 局部代价地图
/local_costmap/published_footprint    ← 机器人轮廓
```

#### 现象判读表


| 观察现象                   | 说明                        | 对应参数                                |
| -------------------------- | --------------------------- | --------------------------------------- |
| 轨迹扇形密集均匀           | 采样充分，正常              | `vx_samples` / `vtheta_samples`         |
| 最优轨迹（绿线）贴近障碍   | 避障权重不足                | 调高`BaseObstacle.scale`（建议 20～50） |
| 机器人横向偏离全局路径     | 路径跟随权重不足            | 调高`PathDist.scale`                    |
| 接近终点不转向、直接冲过   | RotateToGoal 未生效         | 检查`min_speed_theta`，需 > 0           |
| cmd_vel 中 theta 持续抖动  | 震荡                        | 增大`sim_time` 或降低 `vtheta_samples`  |
| 扇形几乎没有向障碍侧的候选 | 正常（BaseObstacle 在过滤） | 若过度保守则降低`BaseObstacle.scale`    |

#### 通关标准

```
✓ RViz 中候选轨迹扇形可见（debug_trajectory_details 生效）
✓ 最优轨迹不进入红色膨胀区
✓ 机器人横向跟随误差 < 0.1 m
✓ 到达终点 0.5 m 内时 cmd_vel 中 vx 趋近 0，omega 接管转向
✓ 全程 cmd_vel 无突变抖动
```

---

## 第四阶段：Recovery Behaviors

### 4.1 min_rotational_vel — 恢复行为最小转速

差速小车原地转向的速度下限，过高会导致恢复行为转动太猛。

- 太高：恢复时转向动作生硬，可能冲过目标角度
- 太低：电机死区内无法启动

建议值：

```yaml
min_rotational_vel: 0.2
max_rotational_vel: 0.6
```

---

## 总结：必调项清单（优先级排序）


| 优先级  | 参数                                     | 当前值 | 建议值 | 原因                       |
| ------- | ---------------------------------------- | ------ | ------ | -------------------------- |
| 🔴 紧急 | `BaseObstacle.scale`                     | 0.1    | 20.0   | 几乎无避障，会撞墙         |
| 🔴 紧急 | `inflation_radius`（global）             | 0.25   | 0.35   | 等于机器人半宽，无安全余量 |
| 🔴 紧急 | `inflation_radius`（local）              | 0.12   | 0.20   | 安全余量仅 19 mm           |
| 🟡 重要 | `expected_planner_frequency`             | 20.0   | 1.0    | 全局规划器跑不到 20 Hz     |
| 🟡 重要 | `general_goal_checker.xy_goal_tolerance` | 0.05   | 0.15   | 里程计漂移会导致终点震荡   |
| 🟡 重要 | `min_y_velocity_threshold`               | 0.5    | 0.001  | 差速车 Y 向速度恒为 0      |
| 🟢 优化 | `cost_scaling_factor`                    | 2.5    | 3.0    | 稍增以改善窄道通行         |
| 🟢 优化 | `PathAlign.forward_point_distance`       | 0.1    | 0.2    | 低速大转弯时朝向对齐效果差 |
| 🟢 优化 | `min_rotational_vel`                     | 0.4    | 0.2    | 恢复动作更平顺             |

---

## 调试工具速查

```bash
# rqt 可视化调参
ros2 run rqt_reconfigure rqt_reconfigure

# 实时修改参数（无需重启）
ros2 param set /controller_server FollowPath.BaseObstacle.scale 20.0

# 查看当前所有参数
ros2 param dump /controller_server

# RViz2 必开的显示项：
#   /local_costmap/costmap              → 查看 costmap 膨胀
#   /local_plan                         → 查看 DWB 局部路径
#   /plan                               → 查看全局路径
#   /local_costmap/published_footprint  → 验证 footprint
```
