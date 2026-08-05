# 删除编译缓存 (需重新编译)
rm -rf build/ install/ log/

# 编译整个功能包
colcon build --packages-select my_bot my_bot_slam my_bot_nav gz_ros2_control --symlink-install
# 编译
colcon build --symlink-install --packages-select my_bot # python编译单个功能包-只需编译一次后续修改不用编译

colcon build --packages-select my_bot # C++ 编译单个功能包

# rqt可视化调参
ros2 run rqt_reconfigure rqt_reconfigure

# 键盘控制 - 设目标速度
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard \
  -p speed:=0.25 -p turn:=0.8

# 1 启动gazebo
ros2 launch my_bot launch_sim.launch.py
# gazebo ctrl+c 结束未杀死执行：pkill -9 -f "^gz sim -s"
# 2 slam（localization 的地图取自 ~/.maps/current_map，可用 map:=<名字> 覆盖）
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization
# 建完图后存成一张新地图（名字不能叫 map）
~/dev_ws/src/my_bot_slam/scripts/save_map.sh <地图名>
# 3 rviz2
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# 4 nav2
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true controller:=neupan
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true \
  params_file:=/home/bingda/dev_ws/src/my_bot_nav/config/sim/nav2_params_neupan_smac2d.yaml
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true map:=my_world_8m \
  params_file:=/home/bingda/dev_ws/src/my_bot_nav/config/sim/nav2_params_neupan_smac2d.yaml
# 单独起禁行区流水线（一般不用，nav.launch.py 已包含）
ros2 launch my_bot_nav keepout.launch.py use_sim_time:=true
# 任务层（包含nav2）
ros2 launch my_bot_task task.launch.py use_sim_time:=true

# 5 app 后端（浏览器 http://127.0.0.1:8080）—— 详见 docs/APP/速查.md
cd ~/ros_flutter_gui && sh ./start.sh

# 6 里程计
./src/my_bot/python/monitor.py

# 查看资源占用
# 右上 CPU 看算力负载，右下 Proc 看具体是哪些进程在占资源，左上 Mem/Disks 看内存与磁盘，左下 Net 看网络流量
btop

# NeuPAN发布目标
ros2 launch my_bot_nav neupan.launch.py use_sim_time:=true   config_file:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/sim/neupan_sim.yaml   model:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/common/neupan/diff_mybot.bin   control_rate:=20.0 replan_rate:=0.0 robot_radius:=0.11 simplify_tolerance:=0.05

# NeuPAN调试车不动
ros2 launch my_bot_nav neupan.launch.py use_sim_time:=true   config_file:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/sim/neupan_sim.yaml   model:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/common/neupan/diff_mybot.bin   cmd_vel_topic:=/neupan_cmd_vel_dry

# 调试全局规划
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.6, y: -0.4, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose   "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.8, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose   "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 0.8, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose   "{goal: {header: {frame_id: 'map'}, pose: {position: {x: -1.5, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"

# 循环发布，每秒执行一次
while true; do
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.8, z: 0.0}, orientation: {w: 1.0}}}}"
  sleep 1
done

# nav2 发布目标
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.4, y: -0.2, z: 0.0}, orientation: {w: 1.0}}}"

# nav2 发布目标 - action(带反馈)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# 取消当前导航
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" &  # 占位
# 或更直接：
ros2 topic pub --once /behavior_server/cancel_all_goals action_msgs/msg/GoalInfo "{}"


# Begin navigating from current location


# direct 模式 —— 你说的那种单独跑 nav 的情况
ros2 run my_bot_task baseline_metrics.py --ros-args \
  -p use_sim_time:=true -p repeat:=2

# task 模式 —— 起了 task.launch.py 时
ros2 run my_bot_task baseline_metrics.py --ros-args \
  -p use_sim_time:=true -p repeat:=3
