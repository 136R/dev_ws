#!/bin/bash
# 回放 bag 建图并评估一组建图参数（车不动）。在另一个 ROS_DOMAIN_ID 里跑，不和正在跑的仿真抢话题。
# 用法: ros2 run my_bot_nav map_group.sh <标签> '<建图参数覆盖 JSON>' [bag 名，默认 route]
#   参数 = config/mapping/mapper_params_sim.yaml + 覆盖。RELAY=truth 时用真值当里程计（做"完美位姿"对照，配 '{"use_scan_matching":false}'）
# 结果: ${MAP_REPLAY_DIR:-/tmp/map_replay}/runs/<标签>/ 下 map_eval.json（地图对 sdf）、size.json（位姿图大小、slam CPU 秒）、
#       loc.log（建图过程 map→base 对真值）、grid.pgm/yaml、位姿图。回放是确定的，每组跑 1 次就够。
W=${MAP_REPLAY_DIR:-/tmp/map_replay}
export ROS_DOMAIN_ID=${REPLAY_DOMAIN:-77}
label=$1; ov=${2:-'{}'}; bag=$W/bags/${3:-route}; R=$W/runs/$label
[ -d $bag ] || { echo "没有 bag: $bag"; exit 1; }
rm -rf $R; mkdir -p $R; cd $R
base=$(ros2 pkg prefix my_bot_slam)/share/my_bot_slam/config/mapping/mapper_params_sim.yaml
python3 - "$ov" "$base" > params.yaml <<'PY'
import json, sys, yaml
d = yaml.safe_load(open(sys.argv[2]))
d['slam_toolbox']['ros__parameters'].update(json.loads(sys.argv[1]))
print(yaml.safe_dump(d))
PY
relay=odom_tf_relay.py; [ "$RELAY" = truth ] && relay=truth_tf_relay.py
setsid bash -c "ros2 run my_bot_nav $relay & ros2 run slam_toolbox async_slam_toolbox_node --ros-args -r __node:=slam_toolbox --params-file $R/params.yaml -p use_sim_time:=true -p mode:=mapping > $R/slam.log 2>&1 & wait" > /dev/null 2>&1 < /dev/null &
sleep 1; sid=$(ps -o sid= -p $! | tr -d ' '); [ -z "$sid" ] && sid=$!
sleep 4
ros2 run my_bot_nav loc_error_test.py $label watch --duration 1000 --stop-file $R/done > loc.log 2>&1 &
lp=$!
sleep 2
ros2 bag play $bag > play.log 2>&1
sleep 3; touch $R/done; wait $lp
ros2 run my_bot_nav map_eval.py --topic --out $R/map_eval.json --save $R/grid > map_eval.log 2>&1
timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$R/$label'}" > /dev/null 2>&1
sp=$(ps -eo pid,sid,cmd | awk -v s=$sid '$2==s && /lib\/slam_toolbox\/async_slam_toolbox_node/ {print $1}' | head -1)
cpu=$(awk -v hz=$(getconf CLK_TCK) '{print ($14+$15)/hz}' /proc/$sp/stat 2>/dev/null)
kill -TERM -- -$sid 2>/dev/null; sleep 2; kill -KILL -- -$sid 2>/dev/null
pg=$(stat -c %s $R/$label.posegraph 2>/dev/null); dt=$(stat -c %s $R/$label.data 2>/dev/null)
echo "{\"cpu_s\": ${cpu:-null}, \"posegraph_kb\": $(( ${pg:-0} / 1024 )), \"data_kb\": $(( ${dt:-0} / 1024 ))}" > $R/size.json
echo "== $label $ov ($bag)"; cat $R/map_eval.json; echo; cat $R/size.json; grep -E "^(all|turning|still)" loc.log
