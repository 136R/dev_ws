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
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/bingda/dev_ws/src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true

ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping

# rviz2
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# slam_toolbox 加载已知地图定位
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/bingda/dev_ws/src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
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
# 2 slam
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=mapping
ros2 launch my_bot_slam slam.launch.py use_sim_time:=true mode:=localization
# 3 rviz2
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# 4 nav2
ros2 launch my_bot_nav nav.launch.py use_sim_time:=true
# 5 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard \
  -p speed:=0.2 -p turn:=0.6
# 6 里程计
./src/my_bot/python/monitor.py

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
