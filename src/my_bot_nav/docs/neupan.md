# NeuPAN 局部控制器

NeuPAN 作为 **`nav2_core::Controller` 插件**接进 Nav2，由 `controller_server` 当作 `FollowPath` 加载。
全局路径经 `setPlan()` 传入，NeuPAN 负责跟踪 + 避障；障碍来自 local costmap 的高代价栅格。

```bash
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true controller:=neupan
```

## 代码在哪

**`src/neupan_cpp`（submodule → fork [`136R/neupan_cpp`](https://github.com/136R/neupan_cpp)）**，
不是上游 `zhangkaiyuan007/neupan_cpp`（那边没有 Nav2 插件）。

新机器上务必 `git clone --recursive`，或 clone 后补 `git submodule update --init`；
否则 `src/neupan_cpp` 是空目录，`colcon build` 找不到 `neupan_cpp_ros::NeuPANController`。

fork 相对上游多了三块（commit `c428288`）：

| 改动 | 文件 |
| --- | --- |
| **Nav2 controller 插件**（595 行） | `neupan_cpp_ros/src/neupan_controller.cpp`、`include/neupan_cpp_ros/{neupan_controller,ros_conversions}.hpp`、`neupan_controller_plugin.xml` |
| **retreat-on-stop（后退避障）** | `libneupan/src/neupan_planner.cpp` + 头文件 |
| **A\* 周期重规划**（`replan_rate`） | `neupan_cpp_ros/src/astar_global_node.cpp` |

跟进上游：`cd src/neupan_cpp && git fetch upstream && git merge upstream/main`。

## 两层参数

**第一层：Nav2 插件参数**（在 `config/{sim,hw}/nav2_params_*_neupan.yaml` 的 `FollowPath:` 段）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `plugin` | `neupan_cpp_ros::NeuPANController` | |
| `config_package` / `config_file` | `my_bot_nav` / `config/sim/neupan_sim.yaml` | 第二层参数从这里加载（按包 share 目录解析） |
| `dune_checkpoint` | `config/common/neupan/diff_mybot.bin` | **自训的 DUNE 模型**，和机器人几何绑定 |
| `map_frame` / `base_frame` | `map` / `base_footprint` | |
| `cost_threshold` | `254` | local costmap 里 ≥ 此代价的栅格视为障碍（LETHAL=254、INSCRIBED=253） |
| `obstacle_stride` | `1` | 栅格抽稀步长；调大降负载但漏障碍 |
| `include_initial_path_direction` | `false` | 参考朝向由路径点梯度推断 |

**第二层：NeuPAN 规划器参数**（`config/{sim,hw}/neupan_{sim,hw}.yaml`）—— 见下。

## 调参

`neupan_sim.yaml` 里每个参数都写了"调大/调小的效果"，这里只说**优先级**。

### 必须先对的（不对就别谈调参）

```yaml
robot:
  length: 0.16      # 机器人几何长度(m)
  width:  0.22      # 机器人几何宽度(m)
```

**必须与实车一致。** 碰撞几何靠它，错了要么撞、要么在能过的地方停死。
`dune_checkpoint` 的模型也是按这套几何训的，两者要匹配。

### 作者点名的主要调参项

| 参数 | 位置 | 调大 → | 调小 → |
| --- | --- | --- | --- |
| `collision_threshold` | 顶层 | 更早急停、更安全，窄处易停死 | 敢靠近，撞险↑ |
| `adjust.d_max` / `d_min` | adjust | 距离变量的硬约束上/下界；`d_min` 调大 = 强制离障更远 | |
| `ref_speed` / `robot.max_speed` | | 跑更快，避障余量小 | 更稳更慢 |
| `adjust.q_s` | adjust | 越贴合全局路径 | 允许偏离，给避障让路 |
| `adjust.p_u` | adjust | 更贴 `ref_speed` | 允许变速以避障 |
| `adjust.eta` | adjust | 更保守绕行、离障更远 | 允许贴近，路径更短 |
| `adjust.ro_obs` | adjust | 强惩闯障、更安全 | 可能需更多迭代才收敛 |

### 嫌慢时按这个顺序降

`receding`（MPC 时域）→ `pan.iter_num` → `pan.dune_max_num` / `nrmp_max_num` → `pan.iter_threshold`

代价是看得更短、收敛更粗。

### retreat-on-stop（后退避障）

fork 独有。原版在 `min_distance < collision_threshold` 时**硬停**；开了 backoff 之后，
它会用中性化的参考**重新求解**，得到一个动力学可行、且感知障碍的**后退指令**。

```yaml
backoff:
  enabled: true
  release_distance: 0.2   # 迟滞：退到 min_dist ≥ 此值才恢复前进（≈ 正面退车距离）
  max_cycles: 20          # 后退封顶帧数，超了就硬停、交回 Nav2 的 progress checker
```

> `max_cycles` 必须 **≥ `release_distance` / 0.015** 才退得到目标，否则会被中途截断。
> 默认 `0.2 / 0.015 ≈ 14 < 20` ✓

### 已核对：这些参数 neupan_cpp 不读

`device`、`time_print` —— C++ 版没有 PyTorch，改了无效。
`ipath.curve_style` 只支持 `'line'`，`robot.kinematics` 只支持 `'diff'`。

## 验收

```bash
# 发目标后立刻跑
ros2 run my_bot_nav neupan_metrics.py --ros-args \
  -p goal_x:=1.5 -p goal_y:=1.5 -p arrive_radius:=0.2 -p timeout:=60.0
```

口径：

- 60 秒内到达目标 0.2m 范围内
- Gazebo 中无可见碰撞
- 最小障碍距离不低于约 0.15m
- 允许短暂停让/后退，但不应长期卡死、原地抖动、反复大幅转向

## 排错

| 现象 | 排查 |
| --- | --- |
| `Failed to create controller. Plugin not found` | submodule 没拉（`git submodule update --init`）或没 `colcon build` |
| 窄处停死不动 | `collision_threshold` 太大，或 `robot.length/width` 填大了；也可开 `backoff` |
| 后退退不出来 | `max_cycles` < `release_distance / 0.015`，被截断了 |
| 贴着障碍走 | `adjust.eta` 太小 / `d_min` 太小 |
| 控制频率跟不上 | 按上面"嫌慢时"的顺序降 `receding` / `iter_num` |
