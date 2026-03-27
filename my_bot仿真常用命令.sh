# 编译
colcon build --symlink-install --packages-select my_bot # python编译单个功能包-只需编译一次后续修改不用编译

colcon build --packages-select my_bot # C++ 编译单个功能包

# 启动gazebo 并加载机器人与场地
ros2 launch my_bot launch_sim.launch.py
# slam_toolbox 建图
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/bingda/dev_ws/src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
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