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
# ── 前置检查：先拦住，别等序列化完几十 MB 才炸 ────────────────────────────
# 1) app 后端必须活着 —— 第 2 步的 pgm/tiles 全靠它产出
if ! curl -fsS -m 5 "$BACKEND/getAllMapList" > /dev/null 2>&1; then
  echo "错误: 连不上 app 后端 ($BACKEND)。" >&2
  echo "      本脚本第 2 步要靠它产出 pgm / yaml / topology / tiles。" >&2
  echo "      先起后端： cd ~/ros_flutter_gui && sh ./start.sh" >&2
  echo >&2
  echo "      注：WSL 下后端没跑时可能报 502 而不是 Connection refused —— WSL2 的 localhost" >&2
  echo "      和 Windows 是打通的，请求会漏到 Windows 那边被别的东西接住。502 ≠ 后端出错。" >&2
  exit 1
fi

# 2) /map 必须有发布者（slam 在跑）
# 注意：不能写成 `ros2 topic info /map | grep -q ...` —— grep -q 一匹配就退出，
# 上游 ros2 收到 SIGPIPE 而非零退出，在 `set -o pipefail` 下整条管道被判为失败，
# 于是"匹配成功"反而走进错误分支。先落到变量里再匹配。
MAP_INFO=$(ros2 topic info /map 2>/dev/null || true)
if ! grep -q "Publisher count: [1-9]" <<< "$MAP_INFO"; then
  echo "错误: /map 没有发布者 —— slam 没在跑。" >&2
  echo "      先起： ros2 launch my_bot_slam slam.launch.py mode:=mapping" >&2
  exit 1
fi

# 3) 后端必须正在持续镜像 /map。
#    ~/.maps/map/ 是第 2 步的复制源，由后端订阅 /map 后写入。
#    slam 按 map_update_interval（sim/hw 均为 1.0s）定时重建并发布 /map，
#    机器人静止时照发不误，所以后端的镜像应当一直是新鲜的（实测年龄 ~1s）。
#
#    镜像陈旧 = 后端收不到 /map。最典型的一种：仿真里 gazebo 挂了但 slam 进程还活着 ——
#    use_sim_time 下 /clock 停摆、ROS 定时器永不触发，slam 不再发 /map。
#    这种情况下 /map 仍然【有发布者】，所以上面第 2 条查不出来，只有 mtime 能抓到。
MIRROR="$MAPS_ROOT/map/map.yaml"
STALE_SEC=30           # /map 是 1Hz，30s 留了 30 倍余量
if [[ ! -f "$MIRROR" ]]; then
  echo "错误: 找不到 $MIRROR（app 后端的实时镜像，第 2 步的复制源）。" >&2
  echo "      后端还没订阅到 /map。确认 slam 在跑，或重启后端。" >&2
  exit 1
fi
MIRROR_AGE=$(( $(date +%s) - $(stat -c %Y "$MIRROR") ))
if (( MIRROR_AGE > STALE_SEC )); then
  echo "错误: 后端的镜像 ${MIRROR_AGE}s 没更新了（/map 本该 1Hz 刷新它）。" >&2
  echo "      后端活着，但收不到 /map。最常见的原因：" >&2
  echo "        · 仿真：gazebo 挂了 → /clock 停 → slam 定时器不触发（进程还在，但不再发图）" >&2
  echo "        · 后端订阅的话题不对（gui_app_settings.json 的 MapSubTopic 应为 /map）" >&2
  echo "      查： ros2 topic hz /map        # 应约 1Hz" >&2
  echo "      （照陈旧镜像复制会得到一张过期的地图。）" >&2
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
