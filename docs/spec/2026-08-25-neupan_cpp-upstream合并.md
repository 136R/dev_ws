# 合并 neupan_cpp upstream 两个新提交（7408e4e、68e0130）

> 状态：**已执行、已验收、已推送**（2026-08-26）
> 修订：2026-08-25 二稿 —— 追加「关闭 backoff」，撤下一稿里「首次启用 backoff」的假设与验收。
> 执行结果见文末【执行结果】栏。

## 【目标】

把 upstream `zhangkaiyuan007/neupan_cpp` 在 `bbb3ac6` 之后的两个提交合进我们的 fork，
让 NeuPAN 在配置写错、求解失败、雷达超时时会明确停车并报状态，而不是静默跑一个没做避障的
标称速度；同时保住 fork 独有的 nav2_core 插件与 A* 周期重规划，并把职责与 Nav2 恢复行为
重叠的 backoff 关掉，让「贴到障碍」这件事统一交回 Nav2 BT 与任务层看门狗处理。

## 【事实】

均已验证，附出处。

- upstream 新增 2 提交，26 文件 +680/-56：
  - `7408e4e`（08-17）参数校验 + 求解状态透出 + halt fallback + DUNE 权重内嵌外形
  - `68e0130`（08-24）arrive 锁存 + `replaceInitialPath` 免冷启动
- **我们只跑 nav2_core 插件路线**。`nav2_params_neupan.yaml` / `nav2_params_neupan_smac2d.yaml` /
  `nav2_params_hw_neupan.yaml` 的 `primary_controller` 均为 `neupan_cpp_ros::NeuPANController`；
  仓库内无任何 launch 引用 `astar_global_node` 或 `neupan_node`。
- `git merge-tree bbb3ac6 HEAD upstream/main` → **5 处冲突，4 个文件**，全部是「两边在同一位置
  各加不同字段」的并集式，无逻辑对撞：
  | 文件 | 冲突数 | 我们的 | upstream 的 |
  |---|---|---|---|
  | `libneupan/include/neupan/neupan_planner.hpp` | 1 | `backing` / `active_threshold` | `solved` / `solver_status` / `min_distance=inf` |
  | `neupan_cpp_ros/src/astar_global_node.cpp` | 3 | `replan_rate_` + 定时器 | `goal_tolerance_` + `last_goal_x/y_` |
  | `neupan_cpp_ros/src/neupan_node.cpp` | 1 | 一行日志 | 删 `refresh_initial_path` |
  | `libneupan/src/neupan_planner.cpp` | （merge-tree 报 changed-in-both，实际不重叠） | `backoff` 段解析 | `validate()` |
- `neupan_cpp_ros/src/neupan_controller.cpp`（fork 独有，542 行）upstream 未触碰，**零冲突**。
- upstream 新 `NeuPANPlanner::validate()` 在构造函数里跑，`collision_threshold <= 0` 直接抛异常。
  我们三份配置 `neupan_sim.yaml` / `neupan_sim_test.yaml` / `neupan_hw.yaml` 顶层均为
  `collision_threshold: -1` → **合并后三份全部起不来**。
- `collision_threshold` 的历史（`git log -S` 于 `neupan_sim.yaml`）：
  `42bf9a8` 引入 `-1` → `2a759d9`「实机/仿真接入 nav2 验收无误」改为 **`0.01`** →
  `74e571b` 改回 `-1`，该提交说明写的是「neupan_hw.yaml 与 neupan_sim.yaml 对齐
  （collision_threshold -1 等）」，主题是雷达量程与 Smac2D，未提碰撞阈值的任何现象。

### backoff 的现状与重叠（本次决定关闭它的依据）

- 三份配置的 `backoff` 段均为 `enabled: true` / `release_distance: 0.15` / `max_cycles: 80` /
  `turn_speed: 1.2`。但 `neupan_planner.cpp` 的判据是
  `want_back = min_distance < collision_threshold`（非退避态），
  `collision_threshold = -1` 使其恒为 false → **backoff 自 `74e571b` 起从未触发过**，
  现有全部导航验收都是在没有 backoff 的条件下取得的。
- **Nav2 BT 已有等效恢复行为**：`navigate_to_pose_custom.xml:94`
  `<BackUp backup_dist="0.15" backup_speed="0.025" />`，与 backoff 的
  `release_distance: 0.15` 是同一个数、同一件事。该文件注释已论证过 BackUp 为何排在
  恢复链最后（`Wait 5s` → `Spin 1.57` → `BackUp`）。
- **口径会被改写**：`max_cycles: 80` × `controller_frequency: 20.0`
  （`nav2_params_neupan_smac2d.yaml:86`）= 最长 **4 s** 的后退，全程 controller 发非零速度。
  这段位移会给 `general_goal_checker` 的 `movement_time_allowance: 8.0`
  和任务层 `nav_watchdog` 的 `min_progress_m: 0.15` / `stuck_timeout_sec: 30.0` 充电，
  绕过 BT 那套按实测日志算过的恢复预算。
- backoff 中「侧向障碍改原地转向」的分支（`turn_speed`）与 BT 的
  `<Spin spin_dist="1.57" />` 功能重复；且 Spin 之后 BT 会重规划，而 controller 层转完
  仍是同一条路径。
- `src/my_bot_nav/docs/neupan.md:200` 的 retreat-on-stop 小节记录了 backoff 的设计与
  `max_cycles ≥ release_distance / 0.015` 的约束。

### 其他

- `Info::min_distance` 默认值 upstream 由 `0.0` 改为 `inf`（无障碍点或已到达时）。
- 我们的 `src/my_bot_nav/config/common/neupan/diff_mybot_new.bin` 用旧脚本导出，不含
  `meta.G`/`meta.h`（`grep -c` = 0）→ 新 DUNE 外形校验只打 WARNING，不阻塞。
- 仓库内**没有** `collision_monitor` 节点在跑（仅 `my_bot_nav/package.xml` 有依赖声明）。
  local costmap 用 12 点多边形 footprint + `inflation_radius: 0.17` 做代价，
  但 Nav2 不会替 controller 插件做速度层急停。
- `neupan_controller.cpp:299-301` 现为 `setInitialPath(...) + reset()`，即 Nav2 每次重规划
  都走冷启动。

## 【假设】

- **假设：`74e571b` 把 `collision_threshold` 从 `0.01` 改回 `-1` 是"对齐 hw/sim"时的顺带回退，
  不是因为 `0.01` 跑出了问题。**
  ｜ 依据：该提交说明只说"对齐"，主题是雷达量程与 Smac2D，未提碰撞阈值的任何现象。
  ｜ 验法：只有你知道 —— 当时是否观察到 `0.01` 导致停死/误急停。
  ｜ 建议：**按 `0.01` 做**（已拍板）；若事后回想起当时确有问题，验收 3 会拦住。

> 一稿里的第二条假设（「首次启用 backoff 是可接受的行为变更」）已作废 —— 本次直接关闭
> backoff，不再启用它。

## 【方案】

按依赖顺序，共 6 步。

### 1. 合并 upstream，解 5 处冲突（`src/neupan_cpp/`）

```bash
cd ~/dev_ws/src/neupan_cpp
git checkout -b merge-upstream-0824
git merge upstream/main
```

逐处解法（全部取并集，除非另行说明）：

| 位置 | 解法 |
|---|---|
| `neupan_planner.hpp` `Info` 结构体 | **两边字段全留**：`backing` / `active_threshold` / `solved` / `solver_status`；`min_distance` **采用 upstream 的 `inf` 默认值**（见步骤 2） |
| `neupan_planner.cpp` | `validate()` 与 `backoff` 段解析并存；`validate()` 里的 `collision_threshold <= 0` 一条**保留不删**（配置侧改值来满足它） |
| `astar_global_node.cpp` ×3 | 两边字段全留；**`goal_tolerance_` 去重判据放在 `onGoal()` 入口，不放进 `plan()`** —— 我们已把 `onGoal` 拆成「记目标」+ `plan(goal)`，去重若放进 `plan()` 会把周期重规划整个废掉 |
| `neupan_node.cpp` | 全取 upstream（删 `refresh_initial_path`），我们那一行只是日志 |

### 2. 复核 `min_distance` 默认值变更（`libneupan/src/neupan_planner.cpp`）

改 `inf` 后，无障碍点的周期不再误判为 `0.0 < threshold`。该变更方向对我们有利，采纳；
需确认 `PAN::Output::min_distance` 在无点路径上确实被赋 `inf` 而非留 `0.0`。

### 3. 三份 NeuPAN 配置：`collision_threshold` `-1` → `0.01`

| 文件 | 改动 |
|---|---|
| `src/my_bot_nav/config/sim/neupan_sim.yaml` | `collision_threshold: -1` → `0.01` |
| `src/my_bot_nav/config/sim/neupan_sim_test.yaml` | 同上 |
| `src/my_bot_nav/config/hw/neupan_hw.yaml` | 同上 |

同步更新该行注释：说明这是硬急停阈值，且 `-1` 会被新版 `validate()` 拒绝。

### 4. 三份 NeuPAN 配置：`backoff.enabled` `true` → `false`

同上三个文件。**只改这一个布尔，`release_distance` / `max_cycles` / `turn_speed` 三个值
原样保留**，将来要复活只需改回 `true`。

在 `backoff:` 段上方补一段注释，写明关闭理由（与 BT 的 `BackUp` / `Spin` 职责重叠，
且 controller 层后退会改写 BT 与看门狗的恢复预算），并指向
`navigate_to_pose_custom.xml` 的 `NavigateRecovery` 与 `task_params.yaml` 的 `nav_watchdog`。

> 关闭 backoff 后，`collision_threshold: 0.01` 的行为退回为纯硬急停：
> `min_distance < 0.01` → `info.stop` → controller 发零速，由 Nav2 的
> `movement_time_allowance: 8.0` 判无进度、进恢复链。**本次合并对小车行为的净变更为零。**

### 5. `neupan_controller.cpp:299-301` 改用 `replaceInitialPath()`

```cpp
// 现在
planner_->setInitialPath(neupan_ros::pathMsgToPoints(...));
planner_->reset();
// 改为：有路径且有位姿时走 replace，保留进度与标称
```

治 Nav2 每次全局重规划都让 NeuPAN 冷启动、车顿一下的问题。

### 6. 文档

- `src/my_bot_nav/docs/neupan.md:200` retreat-on-stop 小节：改写为「fork 独有但**默认关闭**」，
  写明关闭理由与重叠关系，保留复活方式。
- 同文件补 `/neupan_diagnostics`、`/neupan_arrive` 两个新话题，以及新增节点参数
  `scan_timeout` / `solver_fail_grace` / `stall_speed` / `stall_timeout`。
- 补一条：`collision_threshold` 必须 > 0，否则新版加载即抛异常。
- **文档只写约束关系与指针，不写具体数值**（`docs/代码与文档规范.md`）。

## 【不做】

- **不删 backoff 代码**。`neupan_planner.cpp` 里的退避逻辑、`Info::backing` /
  `active_threshold` 字段全部保留，只把配置开关置 `false`。删代码是不可逆的，
  而这次关闭的依据是「与 BT 职责重叠」，不是「实现有错」。
- **不动 `backoff` 的另外三个值**（`release_distance` / `max_cycles` / `turn_speed`）。
- **不动 Nav2 BT 的恢复链与看门狗口径**。关闭 backoff 只是让它们回到原本的设计口径，
  不需要重新配平。
- **不重新导出 DUNE 权重**。`diff_mybot_new.bin` 无 `meta` 记录，新校验只 WARN 不阻塞；
  启用外形校验需用新脚本重训导出，属独立任务。
- **不动 `adjust.d_max` / `d_min`**（现为 `0.06` / `0.01`）。upstream 的 `validate()`
  不拒绝这个组合。调它会显著改变绕行行为，不混进本次合并。
- **不启用 `astar_global_node` / `neupan_node`**。这两个节点的冲突只为保持编译通过与
  将来可用，不进入验收。
- **不动 footprint 与 `robot.length/width` 的已知不一致**（costmap 用 12 点多边形，
  NeuPAN 用 0.22×0.22 居中矩形）。既有问题，与本次合并无关。
- **不动 hw 侧实机运行**。本卡验收只在 sim 完成；`neupan_hw.yaml` 同步改值，实机验证另开。

## 【验收】

sim 侧，`nav2_params_neupan_smac2d.yaml` 方案。

1. **能起来**：合并 + 改配置后 `colcon build --packages-select neupan_cpp_ros libneupan
   --symlink-install` 通过，起 Nav2 后 `ros2 node list` 里 controller_server 存活，
   日志无 `neupan config:` 开头的异常。
2. **诊断可见**：controller 日志或 `/neupan_diagnostics` 能读到 `solved` / `solver_status`；
   正常跟踪时 `solved=true`。
   （插件路线下若 `/neupan_diagnostics` 由 `neupan_node` 独占发布，则只验 controller 日志。）
3. **行为零变更**：从起点导航到一个已验证可达的拓扑点，与合并前的基线相比，
   `/cmd_vel` 的 `linear.x` 不出现连续 3 s 以上的 0 值段，且在 60 s 内到达目标 0.2 m 范围内。
   ```bash
   ros2 run my_bot_nav neupan_metrics.py --ros-args \
     -p goal_x:=<x> -p goal_y:=<y> -p arrive_radius:=0.2 -p timeout:=60.0
   ```
4. **backoff 确实没跑**：同一次导航全程，controller 日志里**不出现** `backing` 字样。
   出现即说明 `enabled: false` 没生效或判据读错了配置。
5. **换路径不顿车**：Nav2 全局重规划（`PathExpiringTimer seconds="3"` 触发）时，
   `/cmd_vel` 的 `linear.x` 不出现单帧掉到 0 再恢复的尖峰 —— 这是步骤 5
   `replaceInitialPath` 的直接效果。

## 【异议】

**有一处异议，但不影响方案对错，我仍按你说的做。**

`collision_threshold: 0.01` 是 1 cm。DUNE 算的是机器人外形到障碍点的距离，1 cm 意味着
**几乎已经贴上了才触发**。关掉 backoff 之后它是 controller 层唯一的急停判据，而我们又没有跑
`collision_monitor`，Nav2 也不会替 controller 插件做速度层急停。

也就是说：改成 `0.01` 满足了新 `validate()`，但真实的避障安全边际仍然全部压在
`adjust.d_min` / `d_max`（`0.01` / `0.06`）和 costmap 的 `inflation_radius: 0.17` 上，
`collision_threshold` 这一层是形式上开着、实质上不起作用的。

不做成选择题的理由：`0.01` 是 `2a759d9`「接入 nav2 验收无误」时用过的值，有过通过记录。
真要提高这层防线，该动的是 `adjust.d_min` / `d_max`，那是另一次调参，已列入【不做】。

对**关闭 backoff** 本身：**无异议**，且这是我建议的方向 —— 依据见【事实】的重叠一节，
核心是它从未触发过，而 BT 已有等效的 `BackUp` + `Spin`，在 controller 层再做一遍会把
按实测日志算过的恢复预算悄悄改写掉。

## 执行提示（新会话）

- 先 `cd ~/dev_ws/src/neupan_cpp && git fetch upstream`，确认 `upstream/main` 仍是 `68e0130`；
  若已前进，回来重算冲突。
- fork 的三个提交是 `c607a1d` / `abfe6ec` / `c428288`，基点 `bbb3ac6`。
- 合并分支建议 `merge-upstream-0824`，验收通过后再并回 `main`。
- 验收 3 需要一条**合并前的基线**做对照。若手头没有，先在合并前跑一次同一目标点存下来。
- 涉及实机运动的命令交给用户执行（约定第 15 条）；本卡验收全在 sim，但起 Gazebo 前先按
  CLAUDE.md 首条检查有没有人已经在跑。

---

## 【执行结果】（2026-08-26）

### 落点

| 仓库 | commit | 内容 |
|---|---|---|
| `src/neupan_cpp`（子模块） | `acf5d70` | 合并 upstream，解 5 处冲突 |
| `src/neupan_cpp` | `a299dd0` | `replaceInitialPath` + 求解状态日志 |
| `dev_ws`（父仓库） | `383a378` | 2 份 YAML + 2 份文档 + 子模块指针 |

`merge-upstream-0824` 已快进并回子模块 `main`；两个仓库均已推送 GitHub
（`neupan_cpp` → `a299dd0`，`dev_ws` → `383a378`）。**`pi` 远端（开发板）未推。**

### 验收（sim，`home_4rooms`，HOME → 拓扑点 C，`nav2_params_neupan_smac2d.yaml`）

| # | 验收项 | 结果 |
|---|---|---|
| 1 | 能起来 | ✅ 编译零 warning；`controller_server` 存活，无 `neupan config:`。DUNE 权重无 footprint metadata，新校验按预期只 WARN 不阻塞 |
| 2 | 诊断可见 | ✅ 见下「偏离」第 2 条 —— 改由 controller 日志透出；正常跟踪全程 `solved=true`（零条未求解 WARN） |
| 3 | 行为零变更 | ✅ 15.6 s 到达（限 60 s）；最长 `linear.x` 零值段 2.22 s（限 3 s），该段为 RotationShim 原地转向（`wz` 从 -0.12 拉到满速 -1.2），非 NeuPAN 卡顿 |
| 4 | backoff 没跑 | ✅ 全程零条 `backing`，零条硬急停 |
| 5 | 换路径不顿车 | ✅ 3 次全局重规划，`/cmd_vel` 单帧掉 0 再恢复的尖峰 0 处 |

### 与卡面的偏离（3 处）

1. **`neupan_sim_test.yaml` 不存在**（用户确认是调参后忘删的废弃条目）。
   步骤 3、4 落到 2 个文件而非 3 个。
2. **卡面未列的一项：给 controller 补了 `solved` / `solver_status` 的节流 WARN。**
   不补的话验收 2 在插件路线下无从观测 —— `/neupan_diagnostics` 由 `neupan_node` 独占发布，
   而我们不跑那个节点。该日志仅供观测，不驱动任何停车决策。
   期间发现并修掉一个自造的误报：`forward()` 的 arrive 分支直接返回、`solved` 停在默认
   `false`，到达帧会被误判成求解失败（upstream 的 `neupan_node` 也是先判 `arrive`）。
3. **未做「合并前基线」对照。** 验收 3、5 的判据本身是绝对量（3 s / 60 s / 尖峰计数），
   实测全部大幅通过，基线只在失败时才有诊断价值。

### 假设的落定

【假设】栏那条（`74e571b` 把 `collision_threshold` 改回 `-1` 是顺带回退，不是因为 `0.01`
跑出了问题）——**验收 3 未被拦住**，`0.01` 全程零急停、零停死，假设成立。

### 遗留（不属于本卡，已记进 `my_bot_nav/docs/README.md` 不变量表）

- ⚠️ **卡面【目标】里「求解失败时明确停车」在插件路线下没有兑现。** upstream 的兜底停车
  （连续未求解 / 雷达超时 / 速度停滞）全在 `neupan_node` 里，而我们只跑 controller 插件。
  现在求解失败时插件仍会把没做避障的标称速度发出去，只多了一条日志。补这层兜底属新增机制。
- ⚠️ **hw 侧配置已同步改值但实机未验证。** 实机上唯一的行为变更点是
  `collision_threshold` 从恒不触发的 `-1` 变成会真触发的 `0.01`。
- 本次验收实际用的 DUNE 权重是工作区里那份**未跟踪**的 `diff_mybot.bin`，不在 git 里，
  克隆方拿不到（属用户未完成的模型改名/归档动作，本次没碰）。
