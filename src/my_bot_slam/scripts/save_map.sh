#!/usr/bin/env bash
# 保存一张新地图，产出「一个文件夹 = 一张完整地图」：
#
#   ~/.maps/<name>/
#   ├── <name>.posegraph / .data   ← slam_toolbox 定位（本脚本第 1 步）
#   ├── <name>.yaml / .pgm         ← app 显示 + keepout 掩码（第 2 步）
#   ├── <name>.topology
#   └── tiles/                      ← app 前端瓦片（第 2 步一并生成）
#
# 用法：建图跑完后执行
#   ./save_map.sh <地图名>
#
# 前置：app 后端（~/ros_flutter_gui/start.sh）在运行，且 slam_toolbox 处于
#       **mapping 模式** —— localization 模式会拒绝 serialize_map（而且照样返回 result=0，
#       所以下面靠文件新鲜度判断成败，不能只看返回码）。
# app 后端会把订阅到的实时地图持续镜像进 ~/.maps/map/（默认地图，名字写死为 "map"），
# 本脚本就拿它当复制源 —— 所以正式地图名不能叫 "map"。
set -euo pipefail

NAME="${1:-}"
BACKEND="${BACKEND_URL:-http://127.0.0.1:8080}"
MAPS_ROOT="$HOME/.maps"

if [[ -z "$NAME" ]]; then
  echo "用法: $0 <地图名>" >&2
  exit 1
fi
if [[ "$NAME" == "map" ]]; then
  echo "错误: \"map\" 是 app 保留给实时镜像的目录名，换一个名字。" >&2
  exit 1
fi
if [[ ! "$NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "错误: 地图名只允许字母/数字/下划线/连字符。" >&2
  exit 1
fi
if [[ ! -f "$MAPS_ROOT/map/map.yaml" ]]; then
  echo "错误: 找不到 $MAPS_ROOT/map/map.yaml。" >&2
  echo "      app 后端要在跑、且已订阅到地图话题（它会把地图镜像到这里当复制源）。" >&2
  exit 1
fi

echo "==> [1/3] 序列化 slam_toolbox 位姿图 → $MAPS_ROOT/$NAME/$NAME.posegraph"
mkdir -p "$MAPS_ROOT/$NAME"
POSEGRAPH="$MAPS_ROOT/$NAME/$NAME.posegraph"
STAMP=$(mktemp); trap 'rm -f "$STAMP"' EXIT   # 用来判断 posegraph 是不是这次新写的
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAPS_ROOT/$NAME/$NAME'}" | tail -2

# serialize_map 在 localization 模式下会被拒绝，但**依然返回 result=0**，
# 所以只能靠"文件是否比本次调用更新"来判断真假成功。
if [[ ! -f "$POSEGRAPH" || ! "$POSEGRAPH" -nt "$STAMP" ]]; then
  echo "错误: 位姿图没写出来。" >&2
  echo "      slam_toolbox 必须处于 mapping 模式 —— localization 模式会拒绝 serialize_map" >&2
  echo "      (日志里会有 'Cannot call serialize map in localization mode!')。" >&2
  echo "      请用 slam.launch.py mode:=mapping 建图后再存。" >&2
  exit 1
fi

echo "==> [2/3] 请 app 后端从实时镜像 \"map\" 复制出 $NAME（pgm + yaml + topology + tiles）"
# /saveMapEdit 加载 <name>.yaml 失败时会回退到 source_map_name —— 这正是新建地图的路径。
# obstacle_edits_json 传空对象 = 不加画笔；topology 传一张空拓扑。
TOPOLOGY=$(printf '{"map_name":"%s","map_property":{"support_controllers":[],"support_goal_checkers":["general_goal_checker"]},"points":[],"routes":[]}' "$NAME")
curl -fsS -G "$BACKEND/saveMapEdit" \
  --data-urlencode "session_id=save_map.sh" \
  --data-urlencode "map_name=$NAME" \
  --data-urlencode "source_map_name=map" \
  --data-urlencode "topology_json=$TOPOLOGY" \
  --data-urlencode "obstacle_edits_json={}"
echo

echo "==> [3/3] 设为当前地图"
curl -fsS -G "$BACKEND/setCurrentMap" --data-urlencode "name=$NAME"
echo

echo
echo "完成。$MAPS_ROOT/$NAME/:"
ls -1 "$MAPS_ROOT/$NAME/"
echo
echo "现在可以启动定位与导航（都会自动读 ~/.maps/current_map）："
echo "  ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization"
echo "  ros2 launch my_bot_nav  nav.launch.py  use_sim_time:=true controller:=neupan"
