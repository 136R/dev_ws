#!/bin/bash
# 仿真录建图回放用的 bag：起 Gazebo + 定位 slam（旧图 my_world_40）+ 导航，nav_route_test 跑 1 圈默认路线并录包，然后只关自己起的会话。
# 用法: ros2 run my_bot_nav record_bag.sh <bag 名> ['<FollowPath 参数 JSON>']   例: record_bag.sh slow '{"desired_linear_vel":0.2}'
# 车会动，仅仿真。bag 写到 ${MAP_REPLAY_DIR:-/tmp/map_replay}/bags/<bag 名>。每趟都重启 Gazebo，让里程计从原点开始（map = world）。
W=${MAP_REPLAY_DIR:-/tmp/map_replay}; mkdir -p $W/bags $W/logs
CTRL=${2:-'{}'}
if ps -eo cmd | grep -qE "^gz sim|[s]lam_toolbox_node|[p]lanner_server"; then
  echo "已有 Gazebo / slam / 导航在跑，先停掉再录（避免两套抢话题）"; exit 1
fi
start() { setsid "$@" > $W/logs/rec_$3.log 2>&1 < /dev/null & echo $!; }
sid_of() { ps -o sid= -p $1 | tr -d ' '; }
p1=$(start ros2 launch my_bot launch_sim.launch.py); sleep 25
p2=$(start ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization map:=my_world_40); sleep 8
p3=$(start ros2 launch my_bot_nav nav.launch.py use_sim_time:=true controller:=rpp path_valid_service:=/path_center_check map:=my_world_40); sleep 25
sids="$(sid_of $p3) $(sid_of $p2) $(sid_of $p1)"; echo "会话 $sids"
rm -rf $W/bags/$1
ros2 bag record -o $W/bags/$1 /scan_filtered /odometry/filtered /tf_static /clock /gz/model/my_bot/pose > $W/logs/rec_$1.log 2>&1 &
bp=$!; sleep 4
(cd $W/logs && timeout 600 ros2 run my_bot_nav nav_route_test.py $1 "$CTRL" 1 2>&1 | tail -1)
kill -INT $bp; wait $bp
for s in $sids; do kill -INT -- -$s 2>/dev/null; done; sleep 8
for s in $sids; do pgrep -s $s >/dev/null && kill -TERM -- -$s; done; sleep 3
ros2 bag info $W/bags/$1 | grep -E "Duration"
