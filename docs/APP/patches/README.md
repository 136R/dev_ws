# 前端补丁

app 源码是**上游 clone**（`~/ROS_Flutter_Gui_App`，remote 指向 `chengyangkj/ROS_Flutter_Gui_App`，
push 不上去），不在本仓库里。所以我们只把**自己的那份 diff** 捞进版本控制。

| 补丁 | 基线 commit | 改的文件 |
| --- | --- | --- |
| `0001-tile-refresh.patch` | `88ad742`（支持前端设置瓦片地图样式） | `app/lib/display/tile_map.dart`、`app/lib/page/map_edit_page.dart` |

## 打补丁

```bash
cd ~/ROS_Flutter_Gui_App
git apply --check ~/dev_ws/docs/APP/patches/0001-tile-refresh.patch   # 干跑，不落盘
git apply         ~/dev_ws/docs/APP/patches/0001-tile-refresh.patch
```

断言：`git diff --stat` 应显示 `2 files changed, 17 insertions(+), 24 deletions(-)`。
`--check` 报错就说明上游动过这两个文件，见下面「上游更新后冲突」。

## 改完前端后，重新导出补丁

```bash
cd ~/ROS_Flutter_Gui_App && git diff > ~/dev_ws/docs/APP/patches/0001-tile-refresh.patch
```

> 前提：工作区**只有**你自己的改动。`git status --short` 确认一下，别把调试代码一起导出去。

## 上游更新后冲突

```bash
cd ~/ROS_Flutter_Gui_App
git stash                       # 收好当前改动
git pull
git stash pop                   # 有冲突就手工解，解完重新导出补丁
```

补丁改动很小（两个文件、几十行），冲突基本只会出现在 `TileLayer(...)` 那一段。
真解不开就照着 [../架构与接口.md](../架构与接口.md) 的「设计取舍 3」重新手打 ——
关键只有三条：**key 不带 cacheBuster、URL 带 `?_ts=`、保存成功时 bump `MapTileStyleEpoch`**。

## 这个补丁在解决什么

见 [../架构与接口.md](../架构与接口.md) 的「设计取舍 3：cache-buster 只能放 URL、不能放 key」。
一句话：**key 放了会整图闪烁，URL 不放画笔就不刷新** —— 两个坑必须同时绕开。
