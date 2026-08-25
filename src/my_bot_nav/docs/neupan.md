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

## 编译（实机 arm64 需要带 PIC）

```bash
# 实机（香橙派 arm64）—— 必须带 --cmake-args
colcon build --packages-up-to neupan_cpp_ros --symlink-install \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
```

`libneupan` 是静态库，要被链进 controller 插件的**动态库**（`add_library(neupan_controller SHARED ...)`）。
CMakeLists 里没有显式设 PIC，所以 aarch64 上链接期会报
`relocation R_AARCH64_* ... can not be used when making a shared object`。

**已在板子上实测证实**（2026-07）：不加 `--cmake-args` 时链接器直接拒绝 ——

```
/usr/bin/ld: install/libneupan/lib/libneupan.a(nrmp.cpp.o):
  relocation R_AARCH64_ADR_PREL_PG_HI21 against symbol ... which may bind externally
  can not be used when making a shared object; recompile with -fPIC
```

炸点是 `nrmp.cpp` 里 OsqpEigen / Eigen 的模板实例。

> **WSL / x86_64 上实测不加也能编过**（x86_64 的重定位模型更宽松）。所以这条只在 arm64 上是硬要求，
> 但两边都带着不会有坏处。
>
> 治本：在 `libneupan` 的 CMakeLists 里设 `POSITION_INDEPENDENT_CODE ON`，两边都不用再记这个参数。
> 需要改 fork（`136R/neupan_cpp`）。**待办。**

fork 相对上游多了四块：

| 改动 | 文件 |
| --- | --- |
| **Nav2 controller 插件** | `neupan_cpp_ros/src/neupan_controller.cpp`、`include/neupan_cpp_ros/{neupan_controller,ros_conversions}.hpp`、`neupan_controller_plugin.xml` |
| **retreat-on-stop（后退避障）** —— 代码在，**配置默认关闭** | `libneupan/src/neupan_planner.cpp` + 头文件 |
| **A\* 周期重规划**（`replan_rate`） | `neupan_cpp_ros/src/astar_global_node.cpp` |
| **全局路径按 MPC 步长弧长重采样** | `neupan_cpp_ros/src/neupan_controller.cpp` 的 `resampleByArcLength` |

跟进上游：`cd src/neupan_cpp && git fetch upstream && git merge upstream/main`。

> **合并上游时注意**：`Info` 结构体、`astar_global_node.cpp` 的成员与参数声明是两边都在加字段的
> 位置，冲突基本都能按并集解。唯一有逻辑的一处是上游的 `goal_tolerance_` 目标去重 ——
> 它必须放在 `onGoal()` 入口，**放进 `plan()` 会把 fork 的 `replan_rate_` 周期重规划整个废掉**
> （周期重规划每次都拿同一个目标调 `plan()`，去重会全部拦下）。

### 上游的运行时保护（`neupan_node` 独占）

上游在 `neupan_cpp_ros/src/neupan_node.cpp` 里加了求解失败/雷达超时/速度停滞的兜底停车，
以及 `/neupan_arrive`、`/neupan_diagnostics` 两个话题和 `scan_timeout` / `solver_fail_grace` /
`stall_speed` / `stall_timeout` 四个节点参数。

⚠️ **我们跑的是 controller 插件路线，`neupan_node` 根本没起**，所以这些兜底和话题在本仓库里
**都不生效**。插件侧只把「本帧是否求解成功」以节流 WARN 打进 `controller_server` 日志
（`neupan_controller.cpp` 的 `consecutive_unsolved_`），仅供观测，不驱动任何停车决策 ——
求解失败时插件仍会把没做避障的标称速度发出去。要补这层兜底是独立任务。

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

## DUNE 模型重训

机器人外廓变了（`length` / `width` / `wheelbase` 任一项）、或想验证一遍可复现的训练流程时才需要。用 upstream 的 Python
训练工具训，再导出成本仓库读的 NPTF `.bin`。

环境：upstream 训练脚本在 `~/NeuPAN_src`（workspace 外单独 `git clone hanruihua/NeuPAN`，
`pip install torch cvxpy`）；导出工具在本仓库 `src/neupan_cpp/libneupan/tools/export_dune_weights.py`。

```bash
# 1. 训练配置 —— length/width/wheelbase 三个数怎么从车体外廓算出来，
#    见下方「踩过的坑」第 1 条；这三个值必须与两份 neupan_*.yaml 的 robot 段逐位相同
cd ~/NeuPAN_src/example/dune_train
cat > dune_train_mybot.yaml << 'EOF'
robot:
  kinematics: 'diff'
  name: 'mybot'
  length: 0.22
  width: 0.22
  wheelbase: -0.094

train:
  direct_train: true
  data_size: 100000
  data_range: [-25, -25, 25, 25]
  batch_size: 256
  epoch: 5000
  valid_freq: 250
  save_freq: 500
  lr: 5e-5
  lr_decay: 0.5
  decay_freq: 1500
EOF

cat > dune_train_mybot.py << 'EOF'
from neupan import neupan

if __name__ == '__main__':
    neupan_planner = neupan.init_from_yaml('dune_train_mybot.yaml')
    neupan_planner.train_dune()
EOF

# 2. 训练（默认 CPU，见「踩过的坑」第 2 条——别为了这个模型特意切 GPU）
python3 dune_train_mybot.py
# 终端最后一行 "Complete Training. The model is saved in ..." 打印的才是真实存盘路径（见第 3 条）

# 3. 导出成 neupan_cpp 读的 NPTF 格式
cd ~/dev_ws/src/neupan_cpp/libneupan/tools
python3 export_dune_weights.py \
  ~/NeuPAN_src/example/dune_train/model/<训练打印的实际目录>/model_5000.pth \
  /tmp/diff_mybot_new.bin \
  4   # 边数=4，矩形机器人固定值，别改

# 4. 先测再上线：新建一份临时 yaml 把 config_file/dune_checkpoint 指向新文件（别直接改正式配置），
#    sim 里验收通过后，把旧模型改名归档、新模型占用 config/common/neupan/diff_mybot.bin
#    这个正式名（别原地覆盖——出问题时没法一条命令换回去），
#    并同步改 hw、sim 两份 neupan_*.yaml 的 robot 段（同一台实体机器人，两份必须一致）
#    ⚠️ wheelbase 这一项必须和新 .bin 同时上线：单独给运行时 yaml 加 wheelbase 会让 NRMP
#       的矩形挪位、而旧 .bin 里 DUNE 学的还是原位置，两者不一致比都不改更糟
```

**怎么确认几何真的传进去了：** 启动 Nav2 后看 `controller_server` 的 `robot rect:` 那条 INFO，
它打印的是 `Robot::diffRectangle` 实际算出的矩形四边。`wheelbase` 漏写时缺省取 0 且不报错，
只能靠这条日志分辨——矩形没有按预期偏移就是没生效。

### 踩过的坑

1. **length/width 不是车身壳体的 CAD 尺寸，是整车外廓（含轮子、脚轮、外挂件）的最远点。**
   这个矩形直接就是运行时 NRMP 的硬避障边界，也是训练 DUNE 时的边界采样依据——只用壳体尺寸
   会漏掉轮子/脚轮的外沿。外廓取自 `my_bot/description/robot_core.xacro` 各 link 的 origin +
   geometry（推导过程见 `docs/spec/2026-08-11-实机几何改动同步.md` 的 F11 与「外廓（B5）」表）。

   **矩形不必以 base_link 对称** —— 训练侧 `neupan/robot/robot.py::cal_vertices_from_length_width`
   与运行时侧 `libneupan/src/robot.cpp` 的 `Robot::diffRectangle` 用的是同一个公式
   `start_x = -(length - wheelbase)/2`，所以 `wheelbase` 可以当 x 向偏置用。设外廓相对 base_link
   原点（= 两驱动轮轴线中点）前伸 `f`、后伸 `b`、侧伸 `s`（均取正值），则：

   ```
   length    = f + b
   width     = 2 s
   wheelbase = f - b      ← 车体后偏时为负，本车即是（base_link 在圆盘圆心前方）
   ```

   ⚠️ **`wheelbase` 键在训练 yaml 和运行时 yaml 里都必须写，且值相同。** 缺省是 0，
   缺一边就等于两边在用不同的车体矩形：DUNE 学的边界和 NRMP 约束的边界对不上，
   症状是明明没碰到却停车、或贴着障碍不减速，且不会有任何报错。
   这是拿"轴距"参数当偏置 hack 用（`diff` 运动学并不消费轴距），只有两侧公式一致才成立——
   跟进上游 merge 后要重新核对这两处公式还是不是同一个。

2. **GPU 对这个模型反而更慢，别为了"加速"特意切。** DUNE 是个很小的 MLP（隐藏层 32、5 层
   Linear），upstream README 说训练可以上 GPU 加速，但这个规模下核函数启动 + CPU↔GPU 数据搬运的
   开销比计算本身还大——实测同一份 5000 epoch 配置，CPU ≈51min，GPU(cuda:0) ≈71min。默认就是
   CPU（`neupan/configuration/__init__.py` 硬编码 `torch.device("cpu")`，只有 yaml 顶层显式加
   `device: 'cuda'` 才会切）。

3. **`model_name` 对应的存盘目录会自动加后缀防覆盖。** `train_dune()` 存盘路径是
   `<脚本所在目录>/model/<robot.name>/`；如果这个目录已存在（比如上次训了一半被 Ctrl+C），会自动
   改存到 `<name>_2`、`<name>_3`……**别按 yaml 里写的 name 直接拼路径去导出**，要看训练结束时终端
   打印的 "Complete Training. The model is saved in ..." 那一行的实际路径。

4. **`export_dune_weights.py` 的 `torch.load` FutureWarning 可以忽略。** 提示的是 `weights_only=False`
   相关的 pickle 安全风险，这是 torch 新版本的通用提示，checkpoint 是自己训的就没问题。

## 调参

`neupan_sim.yaml` 里每个参数都写了"调大/调小的效果"，这里只说**优先级**。

### 必须先对的（不对就别谈调参）

`neupan_{sim,hw}.yaml` 的 `robot` 段：`length` / `width` / `wheelbase`（当前值以文件为准）。

**必须与实车一致，且两份 yaml 逐位相同。** 碰撞几何靠它，错了要么撞、要么在能过的地方停死。
`dune_checkpoint` 指向的 `.bin` 也是按这套几何训的，改了几何就必须重训，否则 NRMP 与 DUNE
用的是两个不同的车。这不是车身壳体的 CAD 尺寸，是整车外廓（含轮子/脚轮）的最远点，
三个值的算法见前面「DUNE 模型重训」的「踩过的坑」第 1 条。

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

> ⚠️ **`collision_threshold` 必须 > 0。** 负值或 0 会被 `NeuPANPlanner::validate()`
> （`libneupan/src/neupan_planner.cpp`）拒绝，**加载即抛异常、`controller_server` 起不来**，
> 日志里是 `neupan config:` 开头的一行。理由：停车条件会退化成「外形已经和障碍重叠」，
> 那时已经不可恢复了。同一个 `validate()` 还会拒绝 `d_min < 0`、`d_max < d_min`、
> 以及 `eta > 0` 但 `d_max <= 0`（后者会让 eta 完全买不到东西却看不出来）。

### 嫌慢时按这个顺序降

`receding`（MPC 时域）→ `pan.iter_num` → `pan.dune_max_num` / `nrmp_max_num` → `pan.iter_threshold`

代价是看得更短、收敛更粗。

### retreat-on-stop（后退避障）—— fork 独有，**默认关闭**

原版在 `min_distance < collision_threshold` 时**硬停**；开了 backoff 之后，
它会用中性化的参考**重新求解**，得到一个动力学可行、且感知障碍的**后退指令**；
障碍在侧面时改为原地转向（差速车没有横移自由度，纯前后退对侧向障碍距离改变很小）。

**两份 `neupan_{sim,hw}.yaml` 的 `backoff.enabled` 现在都是 `false`。** 关闭理由：

- **职责与 Nav2 BT 恢复链重叠。** `config/common/bt/navigate_to_pose_custom.xml`
  的 `NavigateRecovery` 已有 `BackUp` 与 `Spin`，`BackUp` 的后退距离与 backoff 的
  `release_distance` 本就是同一件事、同一个量级。
- **会改写恢复预算。** backoff 在 controller 层持续发非零速度，最长 `max_cycles / controller_frequency`
  秒。这段位移会给 `general_goal_checker` 的 `movement_time_allowance` 和 `my_bot_task` 的
  `task_params.yaml` 里 `nav_watchdog` 的 `min_progress_m` / `stuck_timeout_sec` 充电，
  绕过 BT 那套按实测日志算过的预算。
- backoff 转完仍是同一条路径；BT 的 `Spin` 之后会重规划。

关掉之后 `min_distance < collision_threshold` 退回**纯硬急停**（发零速），
由 Nav2 判无进度、进恢复链。

**要复活**：把 `enabled` 改回 `true` 即可，另外三个值原样保留着。复活时检查这条约束：

> `max_cycles` 必须 **≥ `release_distance` / (单帧后退位移)** 才退得到目标，否则会被中途截断。
> 违反的症状：车退了一小段就硬停，日志里 `backing` 帧数正好等于 `max_cycles`。

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
| 窄处停死不动 | `collision_threshold` 太大，或 `robot.length/width` 填大了 |
| `controller_server` 起不来，日志有 `neupan config:` | 配置被 `validate()` 拒了，见「作者点名的主要调参项」下的 ⚠️ |
| 后退退不出来 | `max_cycles` < `release_distance / 0.015`，被截断了 |
| 贴着障碍走 | `adjust.eta` 太小 / `d_min` 太小 |
| 控制频率跟不上 | 按上面"嫌慢时"的顺序降 `receding` / `iter_num` |
