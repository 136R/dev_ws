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

## DUNE 模型重训

机器人 length/width 变了、或想验证一遍可复现的训练流程时才需要。用 upstream 的 Python
训练工具训，再导出成本仓库读的 NPTF `.bin`。

环境：upstream 训练脚本在 `~/NeuPAN_src`（workspace 外单独 `git clone hanruihua/NeuPAN`，
`pip install torch cvxpy`）；导出工具在本仓库 `src/neupan_cpp/libneupan/tools/export_dune_weights.py`。

```bash
# 1. 训练配置 —— robot.length/width 见下方「踩过的坑」第 1 条，不是车身壳体尺寸
cd ~/NeuPAN_src/example/dune_train
cat > dune_train_mybot.yaml << 'EOF'
robot:
  kinematics: 'diff'
  name: 'mybot'
  length: 0.18
  width: 0.22

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
#    sim 里验收通过后再覆盖 config/common/neupan/diff_mybot.bin，
#    并同步改 hw、sim 两份 neupan_*.yaml 的 length/width（同一台实体机器人，两份必须一致）
```

### 踩过的坑

1. **length/width 不是车身壳体的 CAD 尺寸，是"以两驱动轮轴线为中心对称的矩形"要包住的最远点。**
   `neupan/robot/robot.py::cal_vertices_from_length_width` 生成的矩形以旋转中心对称，这个矩形
   直接就是运行时 NRMP 的硬避障边界，也是训练 DUNE 时的边界采样依据——只用车身壳体尺寸会漏掉
   轮子/脚轮的外沿。`my_bot` 的真实外廓（相对 base_link 原点，即两驱动轮轴线）：
   x∈[-0.089,+0.035]（车体前后不对称，脚轮在后）、y∈[-0.101,+0.101]（驱动轮厚度方向决定，
   不是轮子半径方向）。矩形只能对称，所以 length 要取 2×max(前伸,后伸)，多余的一侧就是浪费但安全的余量。

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

```yaml
robot:
  length: 0.18      # 机器人几何长度(m)
  width:  0.22      # 机器人几何宽度(m)
```

**必须与实车一致。** 碰撞几何靠它，错了要么撞、要么在能过的地方停死。
`dune_checkpoint` 的模型也是按这套几何训的，两者要匹配。这不是车身壳体的 CAD 尺寸，
是"以两驱动轮轴线为中心对称的矩形"要包住的最远点（含轮子/脚轮）——细节见下面
「DUNE 模型重训」一节。2026-07 因为这个原因把 length 从 0.16 改到了 0.18。

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
