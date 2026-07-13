# APP（ROS_Flutter_Gui_App）

上位机 GUI：[chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)。
本目录记录**我们这台机器人怎么用它** —— 集成知识，不是 app 本身的说明书。

## 30 秒全局

```
浏览器 (Flutter Web, dist/)
   │  HTTP :8080  ── 瓦片、地图列表、画笔保存
   ▼
ros_gui_backend  (C++, 部署在 ~/ros_flutter_gui/)   ← 不参与 colcon 构建
   │  ROS 话题/服务         │  文件
   ▼                        ▼
 /map /scan /plan ...     ~/.maps/<地图名>/  ←──┐
                                │              │
                          ┌─────┴─────┐        │
                          ▼           ▼        │
                    slam_toolbox   Nav2        │
                    (posegraph)  (keepout 掩码) │
                          └── my_bot_slam / my_bot_nav 的 launch 读 current_map ─┘
```

**耦合面只有两个**：ROS 话题/服务，和 `~/.maps/` 目录。别的都不相干。

- app 源码：`~/ROS_Flutter_Gui_App`（**上游 clone，push 不上去**）
- app 部署：`~/ros_flutter_gui`
- 我们对前端的改动：[patches/](patches/) —— **只有这一个补丁、两个文件**

---

## ⚠️ 不变量

违反了就出错，但**从代码里看不出来**。改动前先读这张表。

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **地图名不能叫 `map`** | 画的禁行区被 slam 实时地图周期性覆盖、悄悄消失 | `default_map_name` 硬编码为 `"map"`（`map_manager.cpp:362`），后端持续把 `MapSubTopic` 镜像进 `~/.maps/map/` |
| **目录名 = 文件名**：`~/.maps/<n>/<n>.yaml` | 路径写岔 → `Failed processing YAML file ... bad file` | `map_manager.cpp:198` |
| **app 的「另存为」不产 posegraph** | 另存的图起 slam localization 时报错找不到 `.posegraph` | 它走 `/saveMapEdit`，只复制栅格（`map_manager.cpp:190-265`）。`slam.launch.py` 会按几何+像素相似度自动认亲补齐 |
| **`serialize_map` 在 localization 模式下会被拒，但仍返回 `result=0`** | 存图静默失败 | 所以 `save_map.sh` 靠**文件新鲜度**判断成败，不看返回码 |
| **cache-buster 只能放 URL、不能放 `key`** | 放 key → 整图闪烁；URL 不放 → 画笔保存后不刷新 | `tile_map.dart` 的 `TileLayer`，见[架构与接口](架构与接口.md#3-cache-buster-只能放-url不能放-key) |
| **配置字段不能带首尾空格** | app 不 trim → TF 查找静默失败 | `gui_app_settings.json` |
| **`gui_app_settings.json` 覆盖 `cfg/config.yaml`** | 改了 yaml 却不生效 | 后端以 `--config-json` 加载 |

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 理解后端有哪些接口、`~/.maps` 的契约、为什么这么设计 | [架构与接口.md](架构与接口.md) |
| 重装/换机/实机 arm64，从零把 app 跑起来 | [复现与部署.md](复现与部署.md) |
| 日常命令（启动、存图、切图、画笔生效） | [速查.md](速查.md) |
| 前端补丁怎么打、上游更新后怎么办 | [patches/](patches/) |

## 不归本目录管

**多地图切换**和**禁行区（keepout）** 的 ROS 侧实现 —— launch、nav2 参数、`save_map.sh`、
`keepout_mask_reloader` —— 都是 ROS 侧能力（不用 app、改用 RViz 也照样成立），在：

- [my_bot_nav/docs/多地图与禁行区.md](../../src/my_bot_nav/docs/多地图与禁行区.md)
- [my_bot_slam/docs/速查.md](../../src/my_bot_slam/docs/速查.md) 的「建图 → 存图」

本目录只管 **app 侧**：配置、前端补丁、构建部署、后端接口。
