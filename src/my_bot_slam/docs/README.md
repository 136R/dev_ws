# my_bot_slam

SLAM Toolbox 的 launch、参数、存图脚本。

## 一分钟全局

```
slam.launch.py mode:=mapping       → async_slam_toolbox_node        （建新图）
slam.launch.py mode:=localization  → localization_slam_toolbox_node （定位，默认）
                                       ↑ 加载 ~/.maps/<current_map>/<name>.posegraph

nav_slam.launch.py                 → slam + nav 一起起（slam_mode:= 转发给上面）
```

参数文件由 `use_sim_time` 二选一：`config/mapper_params_sim.yaml` / `mapper_params_hw.yaml`。
**`mode` 和 `use_sim_time` 由 launch 覆盖，不用去改 yaml 里的同名字段。**

---

## ⚠️ 不变量

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **`serialize_map` 只在 mapping 模式可用，且被拒时仍返回 `result=0`** | 存图**静默失败** —— 服务说成功，磁盘上啥也没有 | localization 节点日志会打 `Cannot call serialize map in localization mode!`。所以 `save_map.sh` 靠**文件新鲜度**判断成败，不看返回码 |
| **`map_file_name` 不在 yaml 里写死** | 手工改 yaml 会被 launch 的注入覆盖，白改 | `slam.launch.py` 按 `map:=` 或 `~/.maps/current_map` 现算，作为 launch 参数注入（优先级高于 yaml） |
| **地图名不能叫 `map`** | 画的禁行区被 app 的实时镜像覆盖 | 见 [docs/APP](../../../docs/APP/) |
| **app「另存为」的图缺 posegraph** | 起 localization 报错找不到 `.posegraph` | `slam.launch.py` 会按几何+像素相似度自动认亲补齐；找不到源图才报错 |
| **`save_map.sh` 需要 app 后端在跑** | 只得半张地图：有 posegraph（能定位），但没有 pgm/tiles（app 显示不出来、keepout 没掩码）。脚本已加前置检查，会在序列化之前就拦住 | 第 2 步靠后端的 HTTP 接口产出 pgm/yaml/topology/tiles |
| **`/map` 是 latched，且只在地图变化时才发布** | 别用 mtime 判断后端镜像新不新鲜 —— 存图时机器人停着不动，镜像很久没更新完全正常。正确判据是「镜像尺寸 == 实时 `/map` 尺寸」 | 实测：机器人静止时 `ros2 topic hz /map` 12 秒收不到一条 |
| **localization 模式下 `/map` 会随扫描累积变化** | 别指望它每次启动都一模一样。掩码自带 origin，Nav2 按世界坐标对齐，所以不影响 keepout | 实测同一张 posegraph 两次启动得到 79×79 与 109×106 |

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 建图 / 定位 / 存图命令 | [速查.md](速查.md) |
| slam_toolbox 参数怎么调 | [建图调参.md](建图调参.md) |
| 地图怎么切、禁行区怎么工作 | [my_bot_nav/docs/多地图与禁行区.md](../../my_bot_nav/docs/多地图与禁行区.md) |
| app 侧（`~/.maps` 契约） | [docs/APP/](../../../docs/APP/) |
