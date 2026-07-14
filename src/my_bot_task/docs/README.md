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
| **`task_params_file:=` 指到一个【不存在】的文件时，launch_ros 会静默忽略它** | 整个 yaml 都不加载 → 节点全用代码里的 `declare_parameter` 默认值，**没有任何报错**。症状：改了 `task_params.yaml` 的 `dwell_sec` 却不生效。**排查：`ros2 param get /task_manager dwell_sec` 和 yaml 对不上，就是它。**（实测踩过） | launch_ros 行为 |
| **`scripts/*.py` 必须有可执行位（755）** | `--symlink-install` 把 install 目录软链回源文件，源文件没 `+x` → `ros2 run` 报 `No executable found` | 仓库现有脚本都是 755 |
| **`home_pose` 不配就【整行注释掉】，不要写成 `[]`** | 空列表在 rclpy 里推断成 `NOT_SET` → 节点启动时抛 `ParameterUninitializedException` 直接崩 | `task_params.yaml` |
| **待命点靠【名字】约定（`HOME`），不是拓扑的 `type` 字段** | 想当然去用 `type: ChargeStation` → app 的 Dart 侧写 `'ChargeStation'`、C++ 后端认 `'ChargingStation'`，**两边字符串对不上**，存盘往返会静默退化成 `NavGoal`（上游 bug，两侧的未知值 fallback 都不报错） | `nav_point.dart:52` vs `topology_map.hpp:18` |
| **app 设置页里的 `NavGoalTopic` 能被用户改** | 改了 → `/goal_pose` 和 `/goal_pose/cancel` **全断，且没有任何报错**。召唤点了没反应 | 后端的 cancel 话题是 `NavGoalTopic + "/cancel"` 拼出来的 |
| **`gui_app_settings.json` 的 `NavToPoseStatusTopic` 必须是 `/task/nav_status`** | 用回默认的 `/navigate_to_pose/_action/status` → 机器人一到达那次导航就 `succeeded`，**「停止导航」按钮当场消失**，用户再也点不到「我倒完了」。反过来，**不起任务层只跑 Nav2 时，app 的按钮和状态会一直空白** —— 那时要改回默认值 | 见下节 |
| **导航点名字必须唯一** | 名字是任务的唯一标识（去重、队列、日志全靠它）。重名的点会**互相覆盖，只有最后一个生效**。（名字本身随便起，人名、中文都行） | `_reload_topology()` 检测到会 error 提示 |
| **地图名不能叫 `map`** | 见 [docs/速查.md](../../../docs/速查.md) 的全局不变量表 | |

---

## 两个端口的关系（**别搞混**）

**你永远只访问 `:8080`。** `:8090` 不是"换了个端口"，是**多开了一个**，网页在后台自己调。

```
浏览器
  │
  ├─ 打开 http://<ip>:8080  ────────► ros_gui_backend (C++)
  │   网页本身、地图瓦片、激光、          ↑ 原样未动
  │   机器人位姿、发导航目标…
  │
  └─ 网页加载后，其 JS 每秒后台 fetch
      http://<ip>:8090/task/status ──► task_manager (本包)
      只为拿任务卡的数据                  ↑ 新开的
```

**降级行为**：任务层没起 / `:8090` 不通 → 网页照常打开、地图照常显示，**只是底部那张
任务卡不出现**。不报错、不白屏。（`task_channel.dart` 里 catch 掉了，故意静默。）

## HTTP 口（:8090）—— 给 app 前端的

app 后端是**固定 schema 的 protobuf 桥**，只推激光/地图/里程计那几种消息，
**消费不了 `/task/status` 这种自定义类型**（加一个类型要同时改 C++ 后端 + `.proto`
+ Dart 生成代码，比直接 HTTP 贵得多）。

所以任务层**自己开一个 HTTP 口**，前端 1Hz 轮询。召唤是低频离散动作，不需要 WebSocket。

| 路由 | 方法 | 用途 |
| --- | --- | --- |
| `/task/status` | GET | 当前状态 JSON（见下） |
| `/task/skip` | POST | **「我倒完了」** = 跳过等待，立刻归位 |
| `/task/cancel_all` | POST | 清空队列 |

CORS 全放行 —— Flutter Web 是从后端的 `:8080` 加载的，打 `:8090` 属于跨域。

> **线程安全**：HTTP handler 跑在**独立线程**，绝不直接调节点方法（会和 spin 里的
> 回调抢状态机）。它只往 `pending_cmds` 里塞一个字符串，由节点自己的 timer 排空 ——
> **状态机永远只在 ROS 线程里动**。

前端对应的代码：`app/lib/provider/task_channel.dart`、`app/lib/page/task_card.dart`。

---

## 「我倒完了」按钮为什么是独立的

app 原有的「停止导航」按钮（左下角）**只在导航中出现**（`main_page.dart:1021` 的可见性
条件是 `navStatus == executing || accepted`）—— 机器人一到达，那次 `NavigateToPose`
就 `succeeded` 了，**按钮当场消失**。所以它天然按不到"跳过等待"。

> 曾经试过把 `NavToPoseStatusTopic` 指到一个自造的话题上，让那个按钮在 WAITING 时
> 也显示。能跑，但**一个按钮被塞了三种含义**（取消导航 / 跳过等待 / 停止归位），
> 用户分不清此刻按下去是什么意思。已废弃 —— 现在按钮语义各自唯一：
>
> - **左下角「停止导航」** = 取消当前导航（只在导航中出现，恢复上游原义）
> - **底部任务卡的「我倒完了」** = 跳过等待（只在 WAITING 时出现）

## cancel 语义

`/goal_pose/cancel`（app「停止导航」）和 `/task/skip`（任务卡「我倒完了」）
**走的是同一套逻辑**：

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
