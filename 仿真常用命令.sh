# 删除编译缓存 (需重新编译)
rm -rf build/ install/ log/

# 编译整个功能包
colcon build --packages-select my_bot my_bot_slam my_bot_nav gz_ros2_control --symlink-install
# 编译
colcon build --symlink-install --packages-select my_bot # python编译单个功能包-只需编译一次后续修改不用编译

colcon build --packages-select my_bot # C++ 编译单个功能包

# 启动gazebo 并加载机器人与场地
ros2 launch my_bot launch_sim.launch.py
# slam_toolbox 建图
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping

# rviz2
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# slam_toolbox 加载已知地图定位
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization
# nav2
ros2 launch my_bot navigation_launch.py params_file:=/home/bingda/dev_ws/src/my_bot/config/nav2_params.yaml use_sim_time:=true

# AMCL 定位模式
ros2 launch nav2_bringup localization_launch.py \ map:=/home/bingda/dev_ws/src/my_bot/config/slam/my_map_save.yaml \ use_sim_time:=true
# nav2
ros2 launch my_bot navigation_launch.py \ params_file:=/home/bingda/dev_ws/src/my_bot/config/nav2_params.yaml \ use_sim_time:=true \ map_subscribe_transient_local:=true


# rqt可视化调参
ros2 run rqt_reconfigure rqt_reconfigure

# 键盘控制 - 设目标速度
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard \
  -p speed:=0.2 -p turn:=0.6

# 里程计查看
./src/my_bot/python/monitor.py

# 1 启动gazebo
ros2 launch my_bot launch_sim.launch.py
# 2 slam（localization 的地图取自 ~/.maps/current_map，可用 map:=<名字> 覆盖）
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization
# 3 rviz2
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# 4 nav2（已内含 keepout 禁行区；keepout:=false 可关掉）
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true controller:=neupan
# 单独起禁行区流水线（一般不用，nav.launch.py 已包含）
ros2 launch my_bot_nav keepout.launch.py use_sim_time:=true

# 5 app 后端（浏览器 http://127.0.0.1:8080）—— 详见 docs/APP/速查.md
cd ~/ros_flutter_gui && sh ./start.sh
# 建完图后存成一张新地图（名字不能叫 map）
~/dev_ws/src/my_bot_slam/scripts/save_map.sh <地图名>
# 画笔改了禁行区、又不想重启 nav2：热加载掩码
ros2 service call /filter_mask_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: '$HOME/.maps/my_map/my_map.yaml'}"

# 5 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard \
  -p speed:=0.2 -p turn:=0.6
# 6 里程计
./src/my_bot/python/monitor.py
# 7.web
cd ~/ros_flutter_gui && sh ./start.sh
cd ~/ros_flutter_gui
sh ./start.sh

# NeuPAN发布目标
ros2 launch my_bot_nav neupan.launch.py use_sim_time:=true   config_file:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/sim/neupan_sim.yaml   model:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/common/neupan/diff_mybot.bin   control_rate:=20.0 replan_rate:=0.0 robot_radius:=0.11 simplify_tolerance:=0.05

# NeuPAN调试车不动
ros2 launch my_bot_nav neupan.launch.py use_sim_time:=true   config_file:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/sim/neupan_sim.yaml   model:=$(ros2 pkg prefix my_bot_nav)/share/my_bot_nav/config/common/neupan/diff_mybot.bin   cmd_vel_topic:=/neupan_cmd_vel_dry

# 调试全局规划
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.4, y: -0.2, z: 0.0}, orientation: {w: 1.0}}}}"

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
