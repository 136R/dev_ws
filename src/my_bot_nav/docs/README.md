# my_bot_nav

Nav2 的 launch、参数、以及禁行区（keepout）流水线。

## 一分钟全局

```
nav.launch.py  ──┬── navigation_launch.py  → Nav2 全家桶
                 └── keepout.launch.py     → filter_mask_server + costmap_filter_info_server
                                              + keepout_mask_reloader（监听画笔保存，热加载掩码）
```

参数四选一（由 `use_sim_time` × `controller` 决定），**全部是 ThetaStarPlanner + 局部控制器**：

| | `controller:=rpp`（默认） | `controller:=neupan` |
| --- | --- | --- |
| **仿真** | `config/sim/nav2_params_rpp.yaml` | `config/sim/nav2_params_neupan.yaml` |
| **实机** | `config/hw/nav2_params_hw_rpp.yaml` | `config/hw/nav2_params_hw_neupan.yaml` |

也可用 `params_file:=<路径>` 直接覆盖。

---

## ⚠️ 不变量

违反了就出错，但从代码里看不出来。

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **局部控制器只有 `rpp` 和 `neupan` 两个** | 照着网上/旧笔记调 MPPI / DWB / Smac 的参数 —— 那些插件**根本没加载** | `nav.launch.py` 的 `_include_nav()` 只会四选一 |
| **`nav.launch.py` 已内含 keepout** | 再单独起一遍 `keepout.launch.py` → 节点重名冲突 | `nav.launch.py` 里 `IncludeLaunchDescription(keepout.launch.py)`，`keepout:=false` 可关 |
| **地图名不能叫 `map`** | 画的禁行区被 app 的实时镜像悄悄覆盖 | 见 [多地图与禁行区.md](多地图与禁行区.md) 与 [docs/APP](../../../docs/APP/) |
| **`keepout_filter` 必须排在 `inflation_layer` 之前** | 禁行区不被膨胀，机器人会贴着禁行区边缘走 | costmap 的 `plugins` 列表按顺序执行 |
| **NeuPAN 的 `robot.length/width` 必须与实车一致** | 碰撞几何错 → 要么撞、要么在能过的地方停死 | `config/sim/neupan_sim.yaml` 的 `robot:` 段 |
| **`Behavior Tree tick rate 100.00 was exceeded` 是无害警告** | 别去查故障 —— 它只说明某次 tick 超过了 `bt_loop_duration: 10`（=100Hz），CPU 争抢所致（开着 app 前端 + Gazebo + neupan 推理时常见）。导航照常成功 | `bt_loop_duration: 10` |

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 启动、发目标、调试命令 | [速查.md](速查.md) |
| 换地图、画禁行区、掩码怎么工作 | [多地图与禁行区.md](多地图与禁行区.md) |
| NeuPAN 怎么接进 Nav2、怎么调、怎么验收 | [neupan.md](neupan.md) |
| costmap / footprint / goal checker 调参 | [调参.md](调参.md) |
| app（上位机 GUI）那一侧 | [docs/APP/](../../../docs/APP/) |
