# ROS_Flutter_Gui_App 集成（禁行区 + 遥控不闪）

上位机用 [ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
（C/S 架构：robot 侧 C++ 后端 `ros_gui_backend` + 浏览器/客户端前端）。本仓库做了两件事：

1. **禁行区**：app 画笔 → `keepout_mask_diff` 节点做差 → Nav2 `KeepoutFilter`。
   ROS 侧实现与启动顺序见 [`my_bot_nav/docs/nav_cheatsheet.md`](../src/my_bot_nav/docs/nav_cheatsheet.md) 的「禁行区 / Keepout」。
2. **遥控不闪**：给前端打了一个补丁，解决小地图遥控时瓦片图层每 5s 重建导致的整图闪烁。

> ⚠️ app 本体不在本仓库（部署在 `~/ros_flutter_gui`，源码克隆在 `~/ROS_Flutter_Gui_App`）。
> 本文记录所有 **app 侧改动**，便于在实机/重装时复现。

---

## 1. app 侧配置（`gui_app_settings.json`）

后端以 `--config-json ./gui_app_settings.json` 加载，**它覆盖 `cfg/config.yaml`**。
仿真验证通过的关键改动（实机话题/帧名按实际栈调整）：

| 字段 | 值 | 原因 |
| --- | --- | --- |
| `MapSubTopic` | `/map_gui` | 不直接订 `/map`。否则 app 每 2s 把 slam 的 `/map` 镜像回磁盘 PGM、抹掉画笔。改订 `/map_gui` 由 `keepout_mask_diff` 一次性转发，app 只镜像一次。 |
| `MapPubTopic` | `/map_manager/map` | 不往 slam 的 `/map` 上抢发布（默认是 `/map`），避免污染 costmap 静态层。 |
| `BaseLinkFrameName` | `base_footprint` | 本车 EKF 输出 `odom→base_footprint`。**注意别带尾空格**，否则 TF 查找失败。 |
| `SpeedCtrlTopic` | `/cmd_vel_keyboard` | 遥控速度接入 TwistMux 的高优先级输入。 |
| `LaserTopic` / `OdomTopic` | `/scan` 或 `/scan_filtered` / `/odometry/filtered` | 按实际栈对齐。 |

> app 设置页填话题/帧名**切勿带尾随空格**——该 app 不会自动 trim。

---

## 2. 前端补丁（消除遥控闪烁）

**现象**：握手柄遥控时地图反复整图空白闪烁，松开恢复。

**根因**（[`app/lib/display/tile_map.dart`](https://github.com/chengyangkj/ROS_Flutter_Gui_App/blob/main/app/lib/display/tile_map.dart)）：
遥控开启时一个 5s 定时器不断改 `_tileCacheBuster`，它同时进了 `TileLayer` 的
`key`（→ 整个瓦片图层销毁重建、清空 fallback 缓存）和 URL 的 `?_ts=`（→ 强制重下）。
本地图很小，前端固定请求 z=13 而瓦片只生成到 z=5，平时靠 flutter_map 低层级放大兜底；
一重建/重下，兜底瓦片被清空 → 整图空白 → 每 5s 闪一次。

**补丁**（`TileLayer(...)` 处）：

```dart
// 改前
key: ValueKey('tile_layer_${_currentMapName}_$_tileCacheBuster'),
urlTemplate: ... '/tiles/{map_name}/{z}/{x}/{y}.png?_ts=$_tileCacheBuster' ...

// 改后：key 去掉 cacheBuster（不再重建整层）、URL 去掉 ?_ts=（不再强制重下）、加 keepBuffer
key: ValueKey('tile_layer_$_currentMapName'),
keepBuffer: 8,
urlTemplate: ... '/tiles/{map_name}/{z}/{x}/{y}.png' ...
```

localization 模式地图静态，本就不需要该动态刷新；去掉后遥控正常、地图不闪。

> `Tile not found z=13` 警告无害（前端请求未生成的高层级，flutter_map 自动用低层级放大），
> 可忽略；补丁后它也不再每 5s 刷屏。

---

## 3. 如何重新构建前端

前端是 **Flutter Web**，构建产物 `dist/` 是**纯静态文件、跨平台**（在浏览器里跑，与服务端 CPU 无关）。
`.pb.dart` 已随仓库提交，改 Dart UI 代码**无需 protoc**。

**环境（仅构建机需要，如本 WSL）**：Flutter 3.29.3 stable（自带 Dart 3.7.x）。
国内建议设镜像：`PUB_HOSTED_URL=https://pub.flutter-io.cn`、`FLUTTER_STORAGE_BASE_URL=https://storage.flutter-io.cn`。

```bash
# 源码：~/ROS_Flutter_Gui_App（已打补丁）
cd ~/ROS_Flutter_Gui_App/app
flutter pub get
flutter build web --release          # 产物：app/build/web

# 部署到运行目录（先备份）
cp -a ~/ros_flutter_gui/dist ~/ros_flutter_gui/dist.bak
rm -rf ~/ros_flutter_gui/dist
cp -a ~/ROS_Flutter_Gui_App/app/build/web ~/ros_flutter_gui/dist
```

部署后：重启后端 `cd ~/ros_flutter_gui && sh ./start.sh`，浏览器 **Ctrl+Shift+R** 硬刷新
（必要时 F12 → Application → Service Workers → Unregister）。

回滚：`rm -rf ~/ros_flutter_gui/dist && mv ~/ros_flutter_gui/dist.bak ~/ros_flutter_gui/dist`

---

## 4. 实机（香橙派 5 Pro / arm64）部署

**机器人侧不需要装 Flutter。** Flutter 只是构建工具；`dist/` 跨平台可直接复用。

1. 机器人侧取 **arm64 后端**：Release 的 `backend-<tag>-arm64.zip`（自带一份**未打补丁**的 `dist`）。
2. 用 WSL 构建好的**已打补丁 `dist/`** 覆盖 arm64 后端的 `dist/`。
3. 配置机器人侧 `gui_app_settings.json`（同第 1 节，话题/帧名按实机栈）。
4. ROS 侧跑 keepout 流水线（`nav2_params_hw.yaml` 加 `keepout_filter` + 启动 `keepout.launch.py`，去掉 `use_sim_time`），详见 nav_cheatsheet。

> 即「Flutter 在哪构建都行（只构建一次），arm64 机器人只跑后端 + 复用 dist」。
