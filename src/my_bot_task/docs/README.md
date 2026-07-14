# my_bot_task

任务层：**召唤 → 上门 → 等待 → 归位**。移动垃圾桶的业务闭环。

Nav2 的 `NavigateToPose` 是无状态的 —— 机器人跑到目标点之后什么也不会发生。
本包在它上面套一层任务生命周期。

## 一分钟全局

```
app 点拓扑点 ──> 后端发 /goal_pose ──> task_manager（独占订阅）
                                          │ 就近匹配拓扑点 → 房间名
                                          │ 排队（FIFO，同名去重）
                                          ▼
                                   NavigateToPose ──> Nav2
```

**bt_navigator 的 `goal_pose` 订阅被 remap 到 `/nav2/goal_pose`**，所以 `/goal_pose`
归任务层独占。**app 前端一行都不用改**就能召唤。

### 状态机

```
[IDLE] ──队列非空──> [NAVIGATING] ──成功──> [WAITING] ──dwell 到 / 用户点完成──┐
   ▲                     │ 失败重试耗尽                                        │
   │                     └──────────────> _advance()  <───────────────────────┘
   │                                          │
   └──── 成功 ──── [RETURNING] <──队列空──────┘
                       │ 取消 / 重试耗尽
                       ▼
                   [STOPPED]  ──新任务进队──> [NAVIGATING]
```

`STOPPED` 不是死状态 —— 新召唤会立刻救活它。

---

## ⚠️ 不变量

违反了就出错，但从代码里看不出来。

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **`/goal_pose` 的订阅者只能有 `task_manager`** | remap 没生效 → 任务层和 Nav2 **各收一份目标**：机器人跑过去了，但不等待、不归位，行为诡异且极难查。**用 `ros2 topic info /goal_pose --verbose` 硬性检查** | `task.launch.py` 传 `goal_pose_topic:=nav2/goal_pose` |
| **仿真必须传 `use_sim_time:=true`** | dwell 走墙钟 —— Gazebo 实时因子不是 1 时，"等 2 分钟"就不是 2 分钟 | `task.launch.py` 把它塞进节点 `parameters` |
| **任务层的参数文件参数叫 `task_params_file`，绝不能叫 `params_file`** | `IncludeLaunchDescription` **不隔离 LaunchConfiguration 作用域** —— 子 launch（`nav.launch.py`）会继承父作用域的同名配置，而它自己的 `DeclareLaunchArgument` 默认值**不会覆盖已被设置的值**。于是 Nav2 把任务参数当成 nav2_params 加载 → `controller_server` 报 `No critics defined for FollowPath` → **整个 bringup 中止**。（实测踩过） | `task.launch.py` |
| **`scripts/*.py` 必须有可执行位（755）** | `--symlink-install` 把 install 目录软链回源文件，源文件没 `+x` → `ros2 run` 报 `No executable found` | 仓库现有脚本都是 755 |
| **`home_pose` 不配就【整行注释掉】，不要写成 `[]`** | 空列表在 rclpy 里推断成 `NOT_SET` → 节点启动时抛 `ParameterUninitializedException` 直接崩 | `task_params.yaml` |
| **待命点靠【名字】约定（`HOME`），不是拓扑的 `type` 字段** | 想当然去用 `type: ChargeStation` → app 的 Dart 侧写 `'ChargeStation'`、C++ 后端认 `'ChargingStation'`，**两边字符串对不上**，存盘往返会静默退化成 `NavGoal`（上游 bug，两侧的未知值 fallback 都不报错） | `nav_point.dart:52` vs `topology_map.hpp:18` |
| **app 设置页里的 `NavGoalTopic` 能被用户改** | 改了 → `/goal_pose` 和 `/goal_pose/cancel` **全断，且没有任何报错**。召唤点了没反应 | 后端的 cancel 话题是 `NavGoalTopic + "/cancel"` 拼出来的 |
| **`gui_app_settings.json` 的 `NavToPoseStatusTopic` 必须是 `/task/nav_status`** | 用回默认的 `/navigate_to_pose/_action/status` → 机器人一到达那次导航就 `succeeded`，**「停止导航」按钮当场消失**，用户再也点不到「我倒完了」。反过来，**不起任务层只跑 Nav2 时，app 的按钮和状态会一直空白** —— 那时要改回默认值 | 见下节 |
| **导航点名字必须唯一** | 名字是任务的唯一标识（去重、队列、日志全靠它）。重名的点会**互相覆盖，只有最后一个生效**。（名字本身随便起，人名、中文都行） | `_reload_topology()` 检测到会 error 提示 |
| **地图名不能叫 `map`** | 见 [docs/速查.md](../../../docs/速查.md) 的全局不变量表 | |

---

## `/goal_pose/cancel` = 「我倒完了」

app 的「停止导航」按钮发 `std_msgs/Empty` 到 `/goal_pose/cancel`。

### ⚠️ 但那个按钮默认【按不到】—— 所以有了 `/task/nav_status`

[`main_page.dart:1021`](../../../../ROS_Flutter_Gui_App/app/lib/page/main_page.dart#L1021) 的可见性条件是：

```dart
visible: navStatus == ActionStatus.executing || navStatus == ActionStatus.accepted,
```

机器人**一到达**，那次 `NavigateToPose` 就 `succeeded` 了 → **按钮当场消失**，
用户根本点不到「我倒完了」。

而 app 订的是 `gui_app_settings.json` 里的 `NavToPoseStatusTopic`（**可配置**）。
所以本节点自己发一个 `action_msgs/GoalStatusArray` 到 **`/task/nav_status`**，
按**任务**的生命周期而不是**单次导航**的生命周期来报：

| 任务状态 | 上报 | 按钮 | 用户看到的含义 |
| --- | --- | --- | --- |
| `NAVIGATING` | `EXECUTING` | 显示 | 取消这次召唤 |
| **`WAITING`** | **`EXECUTING`** | **显示** | **「我倒完了，回去吧」** |
| `RETURNING` | `EXECUTING` | 显示 | 停下 |
| `IDLE` / `STOPPED` | `SUCCEEDED` / `ABORTED` | 隐藏 | 没事干 |

于是按钮的含义**恰好就是下面这张表的语义**，而 **Flutter 一行都不用改**。

> 副作用：app 上那行状态文字在等待时会显示 `executing`。这是对的 —— 任务确实在进行中。

### cancel 语义

**统一规则：cancel = 结束此刻正在做的这件事，然后往下走。不清空队列。**

| 收到时的状态 | 行为 |
| --- | --- |
| `NAVIGATING` | 取消当前导航 → 丢弃该任务 → 去下一个 |
| `WAITING` | **这就是「用户手动点完成」** → 立刻归位，不用等满 dwell |
| `RETURNING` | 取消归位 → `STOPPED`（原地停住） |
| `IDLE` / `STOPPED` | 无事可做 |

要清空整个队列用 `/task/cancel_all`（也是 `std_msgs/Empty`）。

---

## `/task/status` 契约

`std_msgs/String` 装 JSON，QoS **latched**（`transient_local`, depth 1）——
后接的 `ros2 topic echo` 一订阅就能拿到当前状态，不用等下一次变化。

```json
{
  "state": "NAVIGATING",
  "current": {"name": "客厅", "x": 1.54, "y": 1.55, "theta": 0.0},
  "queue": ["厨房"],
  "queue_len": 1,
  "dwell_remaining": 0.0,
  "last_error": "",
  "stamp": 1234.5
}
```

前端要的"前面还有 N 个" = `queue.index(我的房间)`。

> **为什么不建 `my_bot_task_msgs`**：app 后端**消费不了**自定义 ROS 类型 —— 它是固定
> schema 的 protobuf 桥，加一个类型要同时改 C++ 后端 + `.proto` + Dart 生成代码，
> 比改前端还贵。而 `std_msgs/String` + JSON 能被 `ros2 topic echo` 直接看懂。

---

## 拓扑点从哪来

读 `~/.maps/<current_map>/<name>.topology`（**不是**订阅 `/map/topology` ——
那个话题根本没有发布者，`TopologyLiveTopic` 只是后端的一个配置字符串）。

- 在 app 的地图编辑页加导航点，**名字直接改成「客厅」**（默认预填 `NAV_POINT_<n>`，可编辑）
- 加一个名叫 **`HOME`** 的点作为待命位
- 保存后 **1~2 秒内自动热重载，不用重启本节点**（轮询 mtime）
- 文件不存在 / 没有点都**不算错误** —— warn 并继续轮询（你很可能先起节点再去标点）

匹配不到任何拓扑点的目标（比如 RViz 随手点的空地）**不会被拒绝**，会退化成匿名任务
`点(1.20, 3.40)`，照样走完整流程。**这条路径是 RViz 验收的基础。**

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 启动、发目标、调试命令 | [速查.md](速查.md) |
| 调 dwell / 重试 / 待命点 | [../config/task_params.yaml](../config/task_params.yaml)（每个参数都有注释） |
| Nav2 那一侧 | [my_bot_nav/docs/](../../my_bot_nav/docs/) |
| app（上位机 GUI）那一侧 | [docs/APP/](../../../docs/APP/) |
